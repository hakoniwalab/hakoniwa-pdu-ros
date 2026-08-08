from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import dataclass, field
import os
from pathlib import Path
from threading import Event, Lock, Thread
import time
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from hakoniwa_pdu_ros.action_binding import (
    ActionBinding,
    ActionBindingConfig,
    load_action_binding,
)
from hakoniwa_pdu_ros.action_config_generator import (
    GeneratedActionConfigs,
    generate_action_configs,
)
from hakoniwa_pdu_ros.action_mapping import (
    RosActionClientTypeMapper,
    goal_id_from_ros,
)
from hakoniwa_pdu_ros.env_setup import configure_import_paths


@dataclass(frozen=True)
class _BindingRuntime:
    binding: ActionBinding
    mapper: RosActionClientTypeMapper
    typed_action: Any
    ros_client: Any


@dataclass
class _GoalContext:
    action_name: str
    hakoniwa_goal: Any
    runtime: _BindingRuntime
    ros_goal_handle: Any | None = None
    hakoniwa_accepted: bool = False
    cancel_pending: bool = False
    cancel_accepted: bool = False
    terminal: bool = False
    pending_feedback: list[object] = field(default_factory=list)
    pending_result: object | None = None
    pending_result_error: BaseException | None = None

    @property
    def goal_id(self) -> bytes:
        return self.hakoniwa_goal.goal_id


class HakoniwaRosActionClientNode(Node):
    """Hakoniwa Typed Action Servers backed by ROS 2 Action Clients."""

    def __init__(
        self,
        config: ActionBindingConfig,
        generated: GeneratedActionConfigs,
        rpc_library: str | Path,
    ) -> None:
        super().__init__("hakoniwa_pdu_ros_action_client")
        self._config = config
        self._generated = generated
        self._rpc_library = Path(rpc_library).resolve()
        self._callback_group = ReentrantCallbackGroup()
        self._lock = Lock()
        self._contexts: dict[tuple[str, bytes], _GoalContext] = {}
        self._bindings: dict[str, _BindingRuntime] = {}
        self._raw_servers: list[Any] = []
        self._typed_servers: list[Any] = []
        self._ros_clients: list[Any] = []
        self._stop_requested = Event()
        self._poll_thread: Thread | None = None
        self._closed = False

        try:
            self._initialize_runtime()
        except BaseException:
            self._close_runtime()
            raise

    def _initialize_runtime(self) -> None:
        from hakoniwa_pdu_rpc import ActionServer, make_typed_action_server

        resolved_by_name = {
            action.binding.hakoniwa_action: action
            for action in self._generated.actions
        }
        expected = {binding.hakoniwa_action for binding in self._config.bindings}
        if set(resolved_by_name) != expected:
            raise ValueError("Resolved actions do not match the Action Binding")

        resolved_config = self._generated.output_dir / "resolved-action.json"
        endpoint_config = self._generated.output_dir / "endpoints.json"
        bindings_by_node: dict[str, list[ActionBinding]] = defaultdict(list)
        for binding in self._config.bindings:
            bindings_by_node[binding.server_endpoint.node_id].append(binding)

        for node_id, bindings in bindings_by_node.items():
            raw_server = ActionServer(
                self._rpc_library,
                node_id,
                resolved_config,
                endpoint_config,
                self._config.action.delta_time_usec,
                self._config.action.time_source_type,
            )
            raw_server.start()
            typed_server = make_typed_action_server(
                raw_server,
                resolved_config,
            )
            self._raw_servers.append(raw_server)
            self._typed_servers.append(typed_server)

            for binding in bindings:
                resolved = resolved_by_name[binding.hakoniwa_action]
                mapper = RosActionClientTypeMapper.load(
                    binding.ros_type,
                    resolved.pdu_action_type,
                )
                if mapper.ros_action_type is None:
                    raise ValueError(
                        f"ROS Action type is unavailable: {binding.ros_type}"
                    )
                ros_client = ActionClient(
                    self,
                    mapper.ros_action_type,
                    binding.ros_name,
                    callback_group=self._callback_group,
                )
                runtime = _BindingRuntime(
                    binding=binding,
                    mapper=mapper,
                    typed_action=typed_server.action(binding.hakoniwa_action),
                    ros_client=ros_client,
                )
                self._bindings[binding.hakoniwa_action] = runtime
                self._ros_clients.append(ros_client)
                self.get_logger().info(
                    "action client ready: "
                    f"ros_name={binding.ros_name} "
                    f"ros_type={binding.ros_type} "
                    f"hakoniwa_action={binding.hakoniwa_action} "
                    f"pdu_type={resolved.pdu_action_type} "
                    f"slots={binding.slot_count}"
                )

        self._poll_thread = Thread(
            target=self._poll_loop,
            name="hakoniwa-action-server-poll",
            daemon=True,
        )
        self._poll_thread.start()

    def _poll_loop(self) -> None:
        while not self._stop_requested.is_set():
            delivered = False
            for typed_server in self._typed_servers:
                try:
                    event = typed_server.poll()
                except BaseException as error:
                    if not self._stop_requested.is_set():
                        self.get_logger().error(
                            f"Hakoniwa Action Server poll failed: {error}"
                        )
                    continue
                if event.event.name == "NONE":
                    continue
                delivered = True
                self._dispatch_event(event)
            if not delivered:
                time.sleep(0.001)

    def _dispatch_event(self, event: Any) -> None:
        runtime = self._bindings.get(event.action_name)
        if runtime is None:
            self.get_logger().error(
                f"Hakoniwa event references unknown Action: {event.action_name}"
            )
            return
        if event.goal is None:
            self.get_logger().error(
                f"Hakoniwa Action event has no Goal: event={event.event.name}"
            )
            return

        if event.event.name == "GOAL_REQUEST":
            self._handle_goal_request(runtime, event)
        elif event.event.name == "CANCEL_REQUEST":
            self._handle_cancel_request(runtime, event.goal)
        elif event.event.name == "RUNTIME_CANCEL_REQUEST":
            self._handle_runtime_cancel(runtime, event.goal)
        elif event.event.name == "ERROR":
            self.get_logger().error(
                "Hakoniwa Action endpoint reported an error: "
                f"action={event.action_name} goal_id={event.goal.goal_id.hex()}"
            )
        else:
            self.get_logger().error(
                f"Unsupported Hakoniwa Action event: {event.event.name}"
            )

    def _handle_goal_request(self, runtime: _BindingRuntime, event: Any) -> None:
        key = (event.action_name, event.goal.goal_id)
        with self._lock:
            if key in self._contexts:
                self.get_logger().error(
                    "Duplicate Hakoniwa Goal reached the ROS bridge: "
                    f"action={event.action_name} goal_id={event.goal.goal_id.hex()}"
                )
                self._reject_goal(runtime, event.goal)
                return

        try:
            if not runtime.ros_client.server_is_ready():
                self.get_logger().warning(
                    f"ROS Action Server is unavailable: {runtime.binding.ros_name}"
                )
                self._reject_goal(runtime, event.goal)
                return
            ros_goal = runtime.mapper.goal_to_ros(event.goal_body)
            context = _GoalContext(event.action_name, event.goal, runtime)
            with self._lock:
                self._contexts[key] = context
            future = runtime.ros_client.send_goal_async(
                ros_goal,
                feedback_callback=lambda message: self._on_ros_feedback(
                    context,
                    message,
                ),
            )
            future.add_done_callback(
                lambda completed: self._on_ros_goal_response(context, completed)
            )
        except BaseException as error:
            with self._lock:
                self._contexts.pop(key, None)
            self.get_logger().error(
                "ROS Goal send failed: "
                f"action={event.action_name} goal_id={event.goal.goal_id.hex()} "
                f"error={error}"
            )
            self._reject_goal(runtime, event.goal)

    def _on_ros_goal_response(self, context: _GoalContext, future: Any) -> None:
        try:
            ros_goal_handle = future.result()
            if ros_goal_handle is None or not ros_goal_handle.accepted:
                self._reject_goal(context.runtime, context.hakoniwa_goal)
                self._remove_context(context)
                return

            with self._lock:
                if context.terminal:
                    return
                context.ros_goal_handle = ros_goal_handle
            result_future = ros_goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda completed: self._on_ros_result(context, completed)
            )

            context.runtime.typed_action.accept_goal(context.hakoniwa_goal)
            with self._lock:
                if context.terminal:
                    return
                context.hakoniwa_accepted = True
                pending_feedback = context.pending_feedback
                pending_result = context.pending_result
                pending_result_error = context.pending_result_error
                context.pending_feedback = []
                context.pending_result = None
                context.pending_result_error = None
            for feedback in pending_feedback:
                self._publish_feedback(context, feedback)
            if pending_result_error is not None:
                self.get_logger().error(
                    "ROS Result retrieval failed: "
                    f"action={context.action_name} "
                    f"goal_id={context.goal_id.hex()} "
                    f"error={pending_result_error}"
                )
                self._complete_with_default_abort(context)
            elif pending_result is not None:
                self._deliver_result(context, pending_result)
        except BaseException as error:
            self.get_logger().error(
                "ROS Goal response handling failed: "
                f"action={context.action_name} goal_id={context.goal_id.hex()} "
                f"error={error}"
            )
            self._remove_context(context)
            if context.ros_goal_handle is not None:
                self._cancel_ros_goal_best_effort(context)

    def _on_ros_feedback(self, context: _GoalContext, message: Any) -> None:
        feedback = getattr(message, "feedback", message)
        with self._lock:
            if context.terminal:
                return
            if not context.hakoniwa_accepted:
                context.pending_feedback.append(feedback)
                return
        self._publish_feedback(context, feedback)

    def _publish_feedback(self, context: _GoalContext, feedback: object) -> None:
        try:
            typed_feedback = context.runtime.typed_action.create_feedback()
            context.runtime.mapper.feedback_to_typed(feedback, typed_feedback)
            context.runtime.typed_action.send_feedback(
                context.hakoniwa_goal,
                typed_feedback,
            )
        except BaseException as error:
            self.get_logger().error(
                "ROS Feedback delivery failed: "
                f"action={context.action_name} goal_id={context.goal_id.hex()} "
                f"error={error}"
            )

    def _on_ros_result(self, context: _GoalContext, future: Any) -> None:
        try:
            result = future.result()
        except BaseException as error:
            with self._lock:
                if context.terminal:
                    return
                if not context.hakoniwa_accepted:
                    context.pending_result_error = error
                    return
            self.get_logger().error(
                "ROS Result retrieval failed: "
                f"action={context.action_name} goal_id={context.goal_id.hex()} "
                f"error={error}"
            )
            self._complete_with_default_abort(context)
            return

        with self._lock:
            if context.terminal:
                return
            if not context.hakoniwa_accepted:
                context.pending_result = result
                return
            if (
                result.status == GoalStatus.STATUS_CANCELED
                and context.cancel_pending
                and not context.cancel_accepted
            ):
                context.pending_result = result
                return
        self._deliver_result(context, result)

    def _deliver_result(self, context: _GoalContext, result: Any) -> None:
        status = self._terminal_status(context, result.status)
        if status is None:
            self._complete_with_default_abort(context)
            return
        try:
            typed_result = context.runtime.typed_action.create_result()
            context.runtime.mapper.result_to_typed(result.result, typed_result)
            context.runtime.typed_action.complete(
                context.hakoniwa_goal,
                status,
                typed_result,
            )
        except BaseException as error:
            self.get_logger().error(
                "ROS Result delivery failed: "
                f"action={context.action_name} goal_id={context.goal_id.hex()} "
                f"error={error}"
            )
        finally:
            self._remove_context(context)

    def _terminal_status(self, context: _GoalContext, ros_status: int) -> Any | None:
        from hakoniwa_pdu_rpc import ActionTerminalStatus

        if ros_status == GoalStatus.STATUS_SUCCEEDED:
            return ActionTerminalStatus.SUCCEEDED
        if ros_status == GoalStatus.STATUS_ABORTED:
            return ActionTerminalStatus.ABORTED
        if ros_status == GoalStatus.STATUS_CANCELED:
            if context.cancel_accepted:
                return ActionTerminalStatus.CANCELED
            self.get_logger().error(
                "ROS Goal became CANCELED without an accepted Hakoniwa Cancel: "
                f"action={context.action_name} goal_id={context.goal_id.hex()}"
            )
            return ActionTerminalStatus.ABORTED
        self.get_logger().error(
            "ROS Result has a non-terminal status: "
            f"action={context.action_name} goal_id={context.goal_id.hex()} "
            f"status={ros_status}"
        )
        return None

    def _complete_with_default_abort(self, context: _GoalContext) -> None:
        from hakoniwa_pdu_rpc import ActionTerminalStatus

        try:
            result = context.runtime.typed_action.create_result()
            context.runtime.typed_action.complete(
                context.hakoniwa_goal,
                ActionTerminalStatus.ABORTED,
                result,
            )
        except BaseException as error:
            self.get_logger().error(
                "Default ABORTED Result delivery failed: "
                f"action={context.action_name} goal_id={context.goal_id.hex()} "
                f"error={error}"
            )
        finally:
            self._remove_context(context)

    def _handle_cancel_request(self, runtime: _BindingRuntime, goal: Any) -> None:
        key = (runtime.binding.hakoniwa_action, goal.goal_id)
        with self._lock:
            context = self._contexts.get(key)
            if (
                context is None
                or context.terminal
                or not context.hakoniwa_accepted
                or context.ros_goal_handle is None
                or context.cancel_pending
            ):
                context = None
            else:
                context.cancel_pending = True
        if context is None:
            self.get_logger().warning(
                "Hakoniwa Cancel has no cancelable ROS Goal: "
                f"action={runtime.binding.hakoniwa_action} "
                f"goal_id={goal.goal_id.hex()}"
            )
            self._reject_cancel(runtime, goal)
            return

        try:
            future = context.ros_goal_handle.cancel_goal_async()
            future.add_done_callback(
                lambda completed: self._on_ros_cancel_response(context, completed)
            )
        except BaseException as error:
            with self._lock:
                context.cancel_pending = False
            self.get_logger().error(
                "ROS Cancel send failed: "
                f"action={context.action_name} goal_id={context.goal_id.hex()} "
                f"error={error}"
            )
            self._reject_cancel(runtime, goal)

    def _on_ros_cancel_response(self, context: _GoalContext, future: Any) -> None:
        try:
            response = future.result()
            accepted = self._cancel_response_contains_goal(
                response,
                context.ros_goal_handle,
            )
        except BaseException as error:
            accepted = False
            self.get_logger().error(
                "ROS Cancel response handling failed: "
                f"action={context.action_name} goal_id={context.goal_id.hex()} "
                f"error={error}"
            )

        with self._lock:
            if context.terminal:
                return
        try:
            if accepted:
                context.runtime.typed_action.accept_cancel(
                    context.hakoniwa_goal
                )
            else:
                context.runtime.typed_action.reject_cancel(
                    context.hakoniwa_goal
                )
        except BaseException as error:
            self.get_logger().error(
                "Hakoniwa Cancel Response delivery failed: "
                f"action={context.action_name} goal_id={context.goal_id.hex()} "
                f"accepted={accepted} error={error}"
            )
            return

        with self._lock:
            if context.terminal:
                return
            context.cancel_pending = False
            context.cancel_accepted = accepted
            pending_result = context.pending_result
            context.pending_result = None
        if pending_result is not None:
            self._deliver_result(context, pending_result)

    @staticmethod
    def _cancel_response_contains_goal(response: Any, ros_goal_handle: Any) -> bool:
        expected = goal_id_from_ros(ros_goal_handle.goal_id)
        for goal_info in getattr(response, "goals_canceling", ()):
            try:
                if goal_id_from_ros(goal_info.goal_id) == expected:
                    return True
            except ValueError:
                continue
        return False

    def _handle_runtime_cancel(self, runtime: _BindingRuntime, goal: Any) -> None:
        with self._lock:
            context = self._contexts.get(
                (runtime.binding.hakoniwa_action, goal.goal_id)
            )
        if context is None or context.ros_goal_handle is None:
            self.get_logger().warning(
                "Runtime Cancel has no matching ROS Goal: "
                f"action={runtime.binding.hakoniwa_action} "
                f"goal_id={goal.goal_id.hex()}"
            )
            return
        self._cancel_ros_goal_best_effort(context)

    def _cancel_ros_goal_best_effort(self, context: _GoalContext) -> None:
        try:
            context.ros_goal_handle.cancel_goal_async()
        except BaseException as error:
            self.get_logger().error(
                "ROS Goal cleanup cancel failed: "
                f"action={context.action_name} goal_id={context.goal_id.hex()} "
                f"error={error}"
            )

    def _reject_goal(self, runtime: _BindingRuntime, goal: Any) -> None:
        try:
            runtime.typed_action.reject_goal(goal)
        except BaseException as error:
            self.get_logger().error(
                "Hakoniwa Goal rejection failed: "
                f"action={runtime.binding.hakoniwa_action} "
                f"goal_id={goal.goal_id.hex()} error={error}"
            )

    def _reject_cancel(self, runtime: _BindingRuntime, goal: Any) -> None:
        try:
            runtime.typed_action.reject_cancel(goal)
        except BaseException as error:
            self.get_logger().error(
                "Hakoniwa Cancel rejection failed: "
                f"action={runtime.binding.hakoniwa_action} "
                f"goal_id={goal.goal_id.hex()} error={error}"
            )

    def _remove_context(self, context: _GoalContext) -> None:
        with self._lock:
            context.terminal = True
            self._contexts.pop((context.action_name, context.goal_id), None)

    def destroy_node(self) -> bool:
        self._close_runtime()
        return super().destroy_node()

    def _close_runtime(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop_requested.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=5.0)
            self._poll_thread = None
        with self._lock:
            contexts = list(self._contexts.values())
            self._contexts.clear()
            for context in contexts:
                context.terminal = True
        for context in contexts:
            if context.ros_goal_handle is not None:
                self._cancel_ros_goal_best_effort(context)
        for client in self._ros_clients:
            try:
                client.destroy()
            except BaseException as error:
                self.get_logger().error(
                    f"ROS Action Client destroy failed: {error}"
                )
        self._ros_clients.clear()
        for server in self._raw_servers:
            try:
                server.close()
            except BaseException as error:
                self.get_logger().error(
                    f"Hakoniwa Action Server close failed: {error}"
                )
        self._raw_servers.clear()
        self._typed_servers.clear()


def run(
    config_path: str | Path,
    *,
    output_dir: str | Path | None = None,
    rpc_library: str | Path | None = None,
) -> None:
    configure_import_paths()
    library = rpc_library or os.environ.get("HAKO_PDU_RPC_LIBRARY")
    if not library:
        raise ValueError("Specify --rpc-library or set HAKO_PDU_RPC_LIBRARY")

    generated = generate_action_configs(config_path, output_dir=output_dir)
    config = load_action_binding(config_path)
    rclpy.init()
    node: HakoniwaRosActionClientNode | None = None
    executor: MultiThreadedExecutor | None = None
    try:
        node = HakoniwaRosActionClientNode(config, generated, library)
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    finally:
        if executor is not None and node is not None:
            executor.remove_node(node)
        if node is not None:
            node.destroy_node()
        if executor is not None:
            executor.shutdown(timeout_sec=5.0)
        rclpy.shutdown()


def main() -> None:
    configure_import_paths()
    parser = argparse.ArgumentParser(
        description="Run the Hakoniwa Typed Action Server to ROS Action Client bridge"
    )
    parser.add_argument("--config", required=True, help="Action Binding JSON path")
    parser.add_argument("--output-dir", help="Generated Action config directory")
    parser.add_argument("--rpc-library", help="PDU-RPC shared library path")
    args = parser.parse_args()
    try:
        run(
            args.config,
            output_dir=args.output_dir,
            rpc_library=args.rpc_library,
        )
    except ValueError as error:
        parser.error(str(error))
