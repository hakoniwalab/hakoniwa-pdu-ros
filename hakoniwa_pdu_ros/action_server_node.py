from __future__ import annotations

import argparse
import contextvars
import importlib
import os
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from hakoniwa_pdu_ros.action_binding import (
    ActionBinding,
    ActionBindingConfig,
    load_action_binding,
)
from hakoniwa_pdu_ros.action_client_runtime import (
    ActionClientRuntime,
    ActionGoalSession,
)
from hakoniwa_pdu_ros.action_wire import ActionWire, load_action_wire
from hakoniwa_pdu_ros.env_setup import configure_import_paths


_CURRENT_ROS_GOAL_ID: contextvars.ContextVar[bytes | None] = contextvars.ContextVar(
    "hakoniwa_pdu_ros_goal_id", default=None
)


class _GoalIdAwareActionServer(ActionServer):
    """Expose the ROS Goal UUID to the public goal callback without changing rclpy.

    Jazzy's ActionServer has already taken the SendGoal request, including
    ``goal_id``, before it invokes the user goal callback.  The public callback
    intentionally receives only the user Goal payload.  This narrow override
    preserves that UUID in a ContextVar while delegating the complete lifecycle
    to the upstream implementation.
    """

    async def _execute_goal_request(self, request_header_and_message):
        _, send_goal_request = request_header_and_message
        ros_goal_id = bytes(send_goal_request.goal_id.uuid)
        token = _CURRENT_ROS_GOAL_ID.set(ros_goal_id)
        try:
            return await super()._execute_goal_request(request_header_and_message)
        finally:
            _CURRENT_ROS_GOAL_ID.reset(token)


@dataclass(frozen=True)
class _ResolvedAction:
    binding: ActionBinding
    ros_action_type: type
    wire: ActionWire


class HakoniwaRosActionServerNode(Node):
    """ROS 2 Action Servers backed by the Hakoniwa Action client runtime."""

    def __init__(self, config: ActionBindingConfig, rpc_library: str | Path) -> None:
        super().__init__("hakoniwa_pdu_ros_action_server")
        from hakoniwa_pdu_rpc import ActionClient

        self._config = config
        self._callback_group = ReentrantCallbackGroup()
        self._native_client = ActionClient(
            rpc_library,
            config.runtime.node_id,
            config.runtime.client_name,
            config.runtime.action_config,
            config.runtime.endpoint_config,
            config.runtime.delta_time_usec,
            config.runtime.time_source_type,
        )
        self._runtime = ActionClientRuntime(self._native_client)
        self._runtime.start()
        self._servers: list[ActionServer] = []
        self._lock = threading.Lock()
        self._sessions: dict[bytes, ActionGoalSession] = {}
        self._closed = False

        try:
            for binding in config.actions:
                resolved = _resolve_action(binding)
                server = _GoalIdAwareActionServer(
                    self,
                    resolved.ros_action_type,
                    binding.ros_name,
                    execute_callback=self._make_execute_callback(resolved),
                    goal_callback=self._make_goal_callback(resolved),
                    handle_accepted_callback=self._make_handle_accepted_callback(resolved),
                    cancel_callback=self._make_cancel_callback(resolved),
                    callback_group=self._callback_group,
                )
                self._servers.append(server)
                self.get_logger().info(
                    f"action ready: ros_name={binding.ros_name} "
                    f"ros_type={binding.ros_type} "
                    f"hakoniwa_action={binding.hakoniwa_action} "
                    f"pdu_type={binding.pdu_action_type}"
                )
        except BaseException:
            self._close_runtime()
            raise

    def _make_goal_callback(self, resolved: _ResolvedAction):
        binding = resolved.binding

        def goal_callback(goal_request: object):
            if not self._runtime.is_running():
                self.get_logger().error(
                    f"Action transport is not running: {binding.hakoniwa_action}"
                )
                return GoalResponse.REJECT

            ros_goal_id = _CURRENT_ROS_GOAL_ID.get()
            if ros_goal_id is None or len(ros_goal_id) != 16 or not any(ros_goal_id):
                self.get_logger().error(
                    f"ROS Goal UUID is unavailable or invalid: {binding.ros_name}"
                )
                return GoalResponse.REJECT

            try:
                template = self._native_client.create_goal_buffer(
                    binding.hakoniwa_action
                )
                pdu = resolved.wire.encode_goal(goal_request, template)
                session, response = self._runtime.submit_goal(
                    binding.hakoniwa_action,
                    pdu,
                    ros_goal_id,
                    timeout_usec=binding.goal_response_timeout_msec * 1000,
                )
            except BaseException as error:
                self.get_logger().error(
                    f"goal submission failed: ros={binding.ros_name} "
                    f"hakoniwa={binding.hakoniwa_action} error={error}"
                )
                return GoalResponse.REJECT

            decision = getattr(response, "decision", None)
            if getattr(decision, "name", None) != "ACCEPTED":
                self._runtime.release(ros_goal_id)
                self.get_logger().info(
                    f"goal rejected: ros={binding.ros_name} "
                    f"hakoniwa={binding.hakoniwa_action}"
                )
                return GoalResponse.REJECT

            with self._lock:
                self._sessions[ros_goal_id] = session
            self.get_logger().info(
                f"goal accepted: ros={binding.ros_name} "
                f"hakoniwa={binding.hakoniwa_action} "
                f"goal_id={ros_goal_id.hex()}"
            )
            return GoalResponse.ACCEPT

        return goal_callback

    def _make_handle_accepted_callback(self, resolved: _ResolvedAction):
        binding = resolved.binding

        def handle_accepted(goal_handle: object) -> None:
            ros_goal_id = _ros_goal_id_bytes(goal_handle)
            with self._lock:
                session = self._sessions.get(ros_goal_id)
            if session is None:
                self.get_logger().error(
                    f"accepted ROS Goal has no Hakoniwa session: {binding.ros_name}"
                )
                goal_handle.abort()
                return
            goal_handle.execute()

        return handle_accepted

    def _make_cancel_callback(self, resolved: _ResolvedAction):
        binding = resolved.binding

        def cancel_callback(goal_handle: object):
            ros_goal_id = _ros_goal_id_bytes(goal_handle)
            with self._lock:
                session = self._sessions.get(ros_goal_id)
            if session is None:
                return CancelResponse.REJECT

            try:
                response = self._runtime.cancel(session)
            except BaseException as error:
                self.get_logger().error(
                    f"cancel failed: ros={binding.ros_name} "
                    f"hakoniwa={binding.hakoniwa_action} error={error}"
                )
                return CancelResponse.REJECT

            if response.event.name == "RESULT":
                return CancelResponse.REJECT
            decision = getattr(response, "decision", None)
            return (
                CancelResponse.ACCEPT
                if getattr(decision, "name", None) == "ACCEPTED"
                else CancelResponse.REJECT
            )

        return cancel_callback

    def _make_execute_callback(self, resolved: _ResolvedAction):
        binding = resolved.binding
        ros_action_type = resolved.ros_action_type

        async def execute_callback(goal_handle: object):
            ros_goal_id = _ros_goal_id_bytes(goal_handle)
            with self._lock:
                session = self._sessions.get(ros_goal_id)
            if session is None:
                goal_handle.abort()
                return ros_action_type.Result()

            try:
                while True:
                    event = self._runtime.wait_for(
                        session,
                        {"FEEDBACK", "RESULT", "TIMEOUT", "ERROR"},
                    )
                    if event.event.name == "FEEDBACK":
                        feedback = resolved.wire.decode_feedback(
                            event.pdu, ros_action_type
                        )
                        goal_handle.publish_feedback(feedback)
                        continue
                    if event.event.name == "RESULT":
                        result = resolved.wire.decode_result(
                            event.pdu, ros_action_type
                        )
                        _commit_ros_terminal(goal_handle, event.terminal_status)
                        return result

                    self.get_logger().error(
                        "Action runtime terminated without Result: "
                        f"ros={binding.ros_name} event={event.event.name}"
                    )
                    goal_handle.abort()
                    return ros_action_type.Result()
            except BaseException as error:
                self.get_logger().error(
                    f"Action execution failed: ros={binding.ros_name} "
                    f"hakoniwa={binding.hakoniwa_action} error={error}"
                )
                goal_handle.abort()
                return ros_action_type.Result()
            finally:
                with self._lock:
                    self._sessions.pop(ros_goal_id, None)
                self._runtime.release(session.goal.goal_id)

        return execute_callback

    def destroy_node(self) -> bool:
        self._close_runtime()
        return super().destroy_node()

    def _close_runtime(self) -> None:
        if self._closed:
            return
        self._closed = True
        for server in self._servers:
            server.destroy()
        self._servers.clear()
        self._runtime.close()
        with self._lock:
            self._sessions.clear()


def _resolve_action(binding: ActionBinding) -> _ResolvedAction:
    return _ResolvedAction(
        binding=binding,
        ros_action_type=import_ros_action_class(binding.ros_type),
        wire=load_action_wire(binding.pdu_action_type),
    )


def import_ros_action_class(type_name: str) -> type:
    parts = type_name.split("/")
    if len(parts) != 3 or parts[1] != "action":
        raise ValueError(
            f"ROS Action type must use package/action/Type form: {type_name}"
        )
    package_name, _, action_name = parts
    module = importlib.import_module(f"{package_name}.action")
    return getattr(module, action_name)


def _ros_goal_id_bytes(goal_handle: object) -> bytes:
    goal_id = getattr(goal_handle, "goal_id")
    value = bytes(goal_id.uuid)
    if len(value) != 16:
        raise ValueError("ROS Action Goal UUID must be 16 bytes")
    return value


def _commit_ros_terminal(goal_handle: object, terminal_status: object) -> None:
    name = getattr(terminal_status, "name", None)
    if name == "SUCCEEDED":
        goal_handle.succeed()
    elif name == "CANCELED":
        goal_handle.canceled()
    else:
        goal_handle.abort()


def run(config_path: str | Path, *, rpc_library: str | Path | None = None) -> None:
    configure_import_paths()
    library = rpc_library or os.environ.get("HAKO_PDU_RPC_LIBRARY")
    if not library:
        raise ValueError("Specify --rpc-library or set HAKO_PDU_RPC_LIBRARY")
    config = load_action_binding(config_path)

    rclpy.init()
    node: HakoniwaRosActionServerNode | None = None
    executor: MultiThreadedExecutor | None = None
    try:
        node = HakoniwaRosActionServerNode(config, library)
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
        description="Run ROS 2 Action Servers backed by Hakoniwa Action clients"
    )
    parser.add_argument("--config", required=True, help="Action Binding JSON path")
    parser.add_argument("--rpc-library", help="PDU-RPC shared library path")
    args = parser.parse_args()
    try:
        run(args.config, rpc_library=args.rpc_library)
    except ValueError as error:
        parser.error(str(error))
