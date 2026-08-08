from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import dataclass
import os
from pathlib import Path
from threading import Event, Lock, Thread
import time
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future as RclpyFuture

from hakoniwa_pdu_ros.action_binding import (
    ActionBinding,
    ActionBindingConfig,
    load_action_binding,
)
from hakoniwa_pdu_ros.action_config_generator import (
    GeneratedActionConfigs,
    ResolvedAction,
    generate_action_configs,
)
from hakoniwa_pdu_ros.action_goal_context import ActionGoalContextRegistry
from hakoniwa_pdu_ros.action_mapping import (
    RosActionServerTypeMapper,
    RosGoalTerminalAction,
    terminal_action_for,
)
from hakoniwa_pdu_ros.env_setup import configure_import_paths


@dataclass
class _GoalDecision:
    completed: Event
    accepted: bool = False


@dataclass
class _CancelDecision:
    completed: Event
    accepted: bool = False


@dataclass(frozen=True)
class _BindingRuntime:
    binding: ActionBinding
    mapper: RosActionServerTypeMapper
    typed_action: Any


class HakoniwaRosActionServerNode(Node):
    """ROS Action Servers backed by Hakoniwa Typed Action Clients."""

    def __init__(
        self,
        config: ActionBindingConfig,
        generated: GeneratedActionConfigs,
        rpc_library: str | Path,
    ) -> None:
        super().__init__("hakoniwa_pdu_ros_action_server")
        self._config = config
        self._generated = generated
        self._rpc_library = Path(rpc_library).resolve()
        # rclpy calls handle_accepted_callback immediately after a successful
        # goal_callback. Serializing this short acceptance phase makes the
        # per-Action pending correlation queue deterministic. Goal execution
        # itself is scheduled separately by ServerGoalHandle.execute().
        self._callback_group = MutuallyExclusiveCallbackGroup()
        self._contexts = ActionGoalContextRegistry()
        self._lock = Lock()
        self._goal_decisions: dict[bytes, _GoalDecision] = {}
        self._cancel_decisions: dict[bytes, _CancelDecision] = {}
        self._completion: dict[bytes, RclpyFuture] = {}
        self._early_events: dict[bytes, list[Any]] = defaultdict(list)
        self._deferred_cancel_results: dict[bytes, Any] = {}
        self._raw_clients: list[Any] = []
        self._typed_clients: list[Any] = []
        self._bindings: dict[str, _BindingRuntime] = {}
        self._action_servers: list[ActionServer] = []
        self._stop_requested = Event()
        self._poll_thread: Thread | None = None
        self._closed = False

        try:
            self._initialize_runtime()
        except BaseException:
            self._close_runtime()
            raise

    def _initialize_runtime(self) -> None:
        from hakoniwa_pdu_rpc import ActionClient, make_typed_action_client

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
            bindings_by_node[binding.client_endpoint.node_id].append(binding)

        for client_index, (node_id, bindings) in enumerate(
            bindings_by_node.items()
        ):
            raw_client = ActionClient(
                self._rpc_library,
                node_id,
                f"hakoniwa_pdu_ros_action_{client_index}",
                resolved_config,
                endpoint_config,
                self._config.action.delta_time_usec,
                self._config.action.time_source_type,
            )
            raw_client.start()
            typed_client = make_typed_action_client(
                raw_client,
                resolved_config,
            )
            self._raw_clients.append(raw_client)
            self._typed_clients.append(typed_client)

            for binding in bindings:
                resolved = resolved_by_name[binding.hakoniwa_action]
                mapper = RosActionServerTypeMapper.load(
                    binding.ros_type,
                    resolved.pdu_action_type,
                )
                if mapper.ros_action_type is None:
                    raise ValueError(
                        f"ROS Action type is unavailable: {binding.ros_type}"
                    )
                runtime = _BindingRuntime(
                    binding=binding,
                    mapper=mapper,
                    typed_action=typed_client.action(binding.hakoniwa_action),
                )
                self._bindings[binding.hakoniwa_action] = runtime
                self._action_servers.append(
                    ActionServer(
                        self,
                        mapper.ros_action_type,
                        binding.ros_name,
                        execute_callback=self._make_execute_callback(runtime),
                        goal_callback=self._make_goal_callback(runtime),
                        handle_accepted_callback=(
                            self._make_accepted_callback(runtime)
                        ),
                        cancel_callback=self._make_cancel_callback(runtime),
                        callback_group=self._callback_group,
                    )
                )
                self.get_logger().info(
                    "action ready: "
                    f"ros_name={binding.ros_name} "
                    f"ros_type={binding.ros_type} "
                    f"hakoniwa_action={binding.hakoniwa_action} "
                    f"pdu_type={resolved.pdu_action_type} "
                    f"slots={binding.slot_count} "
                    f"goal_timeout_msec={binding.goal_response_timeout_msec}"
                )

        self._poll_thread = Thread(
            target=self._poll_loop,
            name="hakoniwa-action-poll",
            daemon=True,
        )
        self._poll_thread.start()

    def _make_goal_callback(self, runtime: _BindingRuntime):
        def goal_callback(ros_goal: object) -> GoalResponse:
            goal_id = self._contexts.new_hakoniwa_goal_id()
            decision = _GoalDecision(Event())
            with self._lock:
                self._goal_decisions[goal_id] = decision
            try:
                typed_goal = runtime.typed_action.create_goal()
                runtime.mapper.goal_to_typed(ros_goal, typed_goal)
                hakoniwa_handle = runtime.typed_action.send_goal(
                    typed_goal,
                    goal_id,
                    timeout_usec=(
                        runtime.binding.goal_response_timeout_msec * 1000
                    ),
                )
            except BaseException as error:
                with self._lock:
                    self._goal_decisions.pop(goal_id, None)
                self.get_logger().error(
                    "Hakoniwa Goal send failed: "
                    f"action={runtime.binding.hakoniwa_action} error={error}"
                )
                return GoalResponse.REJECT

            timeout_sec = runtime.binding.goal_response_timeout_msec / 1000.0
            completed = decision.completed.wait(timeout_sec + 0.1)
            with self._lock:
                self._goal_decisions.pop(goal_id, None)
            if not completed or not decision.accepted:
                self.get_logger().warning(
                    "Hakoniwa Goal rejected: "
                    f"action={runtime.binding.hakoniwa_action} "
                    f"goal_id={goal_id.hex()}"
                )
                return GoalResponse.REJECT

            self._contexts.register_hakoniwa_accepted(
                runtime.binding.hakoniwa_action,
                goal_id,
                hakoniwa_handle,
            )
            return GoalResponse.ACCEPT

        return goal_callback

    def _make_accepted_callback(self, runtime: _BindingRuntime):
        def accepted_callback(ros_goal_handle: object) -> None:
            context = self._contexts.bind_ros_accepted(
                runtime.binding.hakoniwa_action,
                ros_goal_handle,
            )
            completion = RclpyFuture(executor=self.executor)
            with self._lock:
                self._completion[context.hakoniwa_goal_id] = completion
                early = self._early_events.pop(
                    context.hakoniwa_goal_id,
                    [],
                )
            ros_goal_handle.execute()
            for event in early:
                self._deliver_goal_event(runtime, context, event)

        return accepted_callback

    def _make_execute_callback(self, runtime: _BindingRuntime):
        async def execute_callback(ros_goal_handle: object) -> object:
            context = self._contexts.find_by_ros(ros_goal_handle.goal_id)
            if context is None:
                self.get_logger().error(
                    "ROS execute callback has no Hakoniwa Goal mapping: "
                    f"action={runtime.binding.hakoniwa_action}"
                )
                ros_goal_handle.abort()
                return runtime.mapper.ros_result_type()
            with self._lock:
                completion = self._completion[context.hakoniwa_goal_id]
            try:
                return await completion
            finally:
                with self._lock:
                    self._completion.pop(context.hakoniwa_goal_id, None)
                    self._early_events.pop(context.hakoniwa_goal_id, None)
                    self._deferred_cancel_results.pop(
                        context.hakoniwa_goal_id,
                        None,
                    )
                self._contexts.remove_by_hakoniwa(context.hakoniwa_goal_id)

        return execute_callback

    def _make_cancel_callback(self, runtime: _BindingRuntime):
        def cancel_callback(ros_goal_handle: object) -> CancelResponse:
            context = self._contexts.find_by_ros(ros_goal_handle.goal_id)
            if (
                context is None
                or context.action_name != runtime.binding.hakoniwa_action
            ):
                self.get_logger().warning(
                    "ROS Cancel has no matching Hakoniwa Goal: "
                    f"action={runtime.binding.hakoniwa_action}"
                )
                return CancelResponse.REJECT

            decision = _CancelDecision(Event())
            with self._lock:
                if context.hakoniwa_goal_id in self._cancel_decisions:
                    self.get_logger().warning(
                        "Duplicate ROS Cancel is already pending: "
                        f"action={runtime.binding.hakoniwa_action} "
                        f"goal_id={context.hakoniwa_goal_id.hex()}"
                    )
                    return CancelResponse.REJECT
                self._cancel_decisions[context.hakoniwa_goal_id] = decision

            try:
                runtime.typed_action.cancel_goal(
                    context.hakoniwa_goal_handle
                )
            except BaseException as error:
                with self._lock:
                    self._cancel_decisions.pop(
                        context.hakoniwa_goal_id,
                        None,
                    )
                self.get_logger().error(
                    "Hakoniwa Cancel send failed: "
                    f"action={runtime.binding.hakoniwa_action} "
                    f"goal_id={context.hakoniwa_goal_id.hex()} "
                    f"error={error}"
                )
                return CancelResponse.REJECT

            # Cancel ResponseにはGoal Response timeoutを流用しない。ROSへ
            # ACCEPTを返した後でHakoniwa側だけREJECTになる、またはROSへ
            # REJECTを返した後でHakoniwa側だけCANCELEDになる分岐を避ける。
            # Resultが先着した場合とshutdown時はpoll/cleanup側が解除する。
            while not decision.completed.wait(0.05):
                if self._stop_requested.is_set():
                    break

            with self._lock:
                self._cancel_decisions.pop(
                    context.hakoniwa_goal_id,
                    None,
                )
            if decision.completed.is_set() and decision.accepted:
                return CancelResponse.ACCEPT

            self.get_logger().warning(
                "Hakoniwa Cancel was not accepted: "
                f"action={runtime.binding.hakoniwa_action} "
                f"goal_id={context.hakoniwa_goal_id.hex()}"
            )
            return CancelResponse.REJECT

        return cancel_callback

    def _poll_loop(self) -> None:
        while not self._stop_requested.is_set():
            delivered = False
            for typed_client in self._typed_clients:
                try:
                    event = typed_client.poll()
                except BaseException as error:
                    if not self._stop_requested.is_set():
                        self.get_logger().error(
                            f"Hakoniwa Action poll failed: {error}"
                        )
                    continue
                if event.event.name == "NONE":
                    continue
                delivered = True
                self._dispatch_event(event)
            self._flush_deferred_cancel_results()
            if not delivered:
                time.sleep(0.001)

    def _dispatch_event(self, event: Any) -> None:
        if event.goal is None:
            self.get_logger().error(
                f"Hakoniwa Action event has no Goal: event={event.event.name}"
            )
            return
        goal_id = event.goal.goal_id
        if event.event.name in {"GOAL_RESPONSE", "TIMEOUT"}:
            with self._lock:
                decision = self._goal_decisions.get(goal_id)
                if decision is not None:
                    decision.accepted = (
                        event.event.name == "GOAL_RESPONSE"
                        and event.decision.name == "ACCEPTED"
                    )
                    decision.completed.set()
            if decision is None:
                self.get_logger().warning(
                    "Late or unknown Hakoniwa Goal Response: "
                    f"action={event.action_name} goal_id={goal_id.hex()}"
                )
            return

        if event.event.name == "CANCEL_RESPONSE":
            with self._lock:
                decision = self._cancel_decisions.get(goal_id)
                if decision is not None:
                    decision.accepted = event.decision.name == "ACCEPTED"
                    decision.completed.set()
            if decision is None:
                self.get_logger().warning(
                    "Late or unknown Hakoniwa Cancel Response: "
                    f"action={event.action_name} goal_id={goal_id.hex()}"
                )
            return

        if event.event.name == "ERROR":
            with self._lock:
                goal_decision = self._goal_decisions.get(goal_id)
                cancel_decision = self._cancel_decisions.get(goal_id)
                if goal_decision is not None:
                    goal_decision.completed.set()
                if cancel_decision is not None:
                    cancel_decision.completed.set()
            if goal_decision is None and cancel_decision is None:
                self.get_logger().error(
                    "Hakoniwa Action error has no pending operation: "
                    f"action={event.action_name} goal_id={goal_id.hex()}"
                )
            return

        if event.event.name == "RESULT":
            # A SUCCEEDED/ABORTED Result may legitimately win while the Client
            # waits for Cancel Response. In that case ROS Cancel is rejected,
            # while the Result continues through the normal delivery path.
            with self._lock:
                cancel_decision = self._cancel_decisions.get(goal_id)
                if cancel_decision is not None:
                    cancel_decision.completed.set()

        runtime = self._bindings.get(event.action_name)
        if runtime is None:
            self.get_logger().error(
                f"Hakoniwa event references unknown Action: {event.action_name}"
            )
            return
        context = self._contexts.find_by_hakoniwa(goal_id)
        if context is None or context.ros_goal_handle is None:
            with self._lock:
                self._early_events[goal_id].append(event)
            return
        if (
            event.event.name == "RESULT"
            and terminal_action_for(event.terminal_status)
            is RosGoalTerminalAction.CANCELED
            and not context.ros_goal_handle.is_cancel_requested
        ):
            # rclpy transitions EXECUTING -> CANCELING only after the cancel
            # callback returns ACCEPT. Hakoniwa may deliver the correctly
            # ordered CANCELED Result before that ROS transition completes.
            # Keep only this cross-runtime delivery gap in the Bridge; the
            # Action protocol state remains owned by both runtimes.
            with self._lock:
                self._deferred_cancel_results[goal_id] = event
            return
        self._deliver_goal_event(runtime, context, event)

    def _flush_deferred_cancel_results(self) -> None:
        ready: list[tuple[_BindingRuntime, Any, Any]] = []
        with self._lock:
            for goal_id, event in list(
                self._deferred_cancel_results.items()
            ):
                context = self._contexts.find_by_hakoniwa(goal_id)
                runtime = self._bindings.get(event.action_name)
                if (
                    context is None
                    or context.ros_goal_handle is None
                    or runtime is None
                ):
                    continue
                if not context.ros_goal_handle.is_cancel_requested:
                    continue
                self._deferred_cancel_results.pop(goal_id, None)
                ready.append((runtime, context, event))
        for runtime, context, event in ready:
            self._deliver_goal_event(runtime, context, event)

    def _deliver_goal_event(
        self,
        runtime: _BindingRuntime,
        context: Any,
        event: Any,
    ) -> None:
        if event.event.name == "FEEDBACK":
            try:
                feedback = runtime.mapper.feedback_to_ros(event.feedback)
                context.ros_goal_handle.publish_feedback(feedback)
            except BaseException as error:
                self.get_logger().error(
                    "Action Feedback conversion failed: "
                    f"action={event.action_name} error={error}"
                )
            return
        if event.event.name != "RESULT":
            return

        try:
            result = runtime.mapper.result_to_ros(event.result)
            terminal = terminal_action_for(event.terminal_status)
            if terminal is RosGoalTerminalAction.SUCCEED:
                context.ros_goal_handle.succeed()
            elif terminal is RosGoalTerminalAction.CANCELED:
                context.ros_goal_handle.canceled()
            else:
                context.ros_goal_handle.abort()
            with self._lock:
                completion = self._completion.get(context.hakoniwa_goal_id)
            if completion is None:
                raise RuntimeError("ROS Goal completion Future is unavailable")
            completion.set_result(result)
        except BaseException as error:
            self.get_logger().error(
                "Action Result delivery failed: "
                f"action={event.action_name} error={error}"
            )

    def destroy_node(self) -> bool:
        self._close_runtime()
        return super().destroy_node()

    def _close_runtime(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop_requested.set()
        with self._lock:
            for decision in self._goal_decisions.values():
                decision.completed.set()
            for decision in self._cancel_decisions.values():
                decision.completed.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=5.0)
            self._poll_thread = None
        for server in self._action_servers:
            server.destroy()
        self._action_servers.clear()
        for client in self._raw_clients:
            try:
                client.close()
            except BaseException as error:
                self.get_logger().error(
                    f"Hakoniwa Action Client close failed: {error}"
                )
        self._raw_clients.clear()
        self._typed_clients.clear()
        with self._lock:
            self._deferred_cancel_results.clear()
        self._contexts.clear()


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
    node: HakoniwaRosActionServerNode | None = None
    executor: MultiThreadedExecutor | None = None
    try:
        node = HakoniwaRosActionServerNode(config, generated, library)
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
        description="Run the ROS Action Server to Hakoniwa Typed Action Client bridge"
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
