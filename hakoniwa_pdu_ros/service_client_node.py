from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import dataclass
import json
import os
from pathlib import Path
from threading import Event, Lock, Thread
import time
from typing import Any

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from hakoniwa_pdu_ros.env_setup import configure_import_paths
from hakoniwa_pdu_ros.service_binding import (
    ServiceBinding,
    ServiceBindingConfig,
    load_service_binding,
)
from hakoniwa_pdu_ros.service_config_generator import (
    GeneratedServiceConfigs,
    ResolvedService,
    generate_service_configs,
)
from hakoniwa_pdu_ros.type_mapper import copy_matching_fields, import_ros_service_class


@dataclass(frozen=True)
class _BindingRuntime:
    binding: ServiceBinding
    resolved: ResolvedService
    typed_service: Any
    ros_client: Any
    ros_service_type: Any


@dataclass
class _CallContext:
    request: Any
    runtime: _BindingRuntime
    ros_future: Any
    started_at: float
    terminal: bool = False


class HakoniwaRosServiceClientNode(Node):
    """Hakoniwa Typed RPC Servers backed by ROS 2 Service Clients."""

    def __init__(
        self,
        config: ServiceBindingConfig,
        generated: GeneratedServiceConfigs,
        rpc_library: str | Path,
    ) -> None:
        super().__init__("hakoniwa_pdu_ros_service_client")
        self._config = config
        self._generated = generated
        self._rpc_library = Path(rpc_library).resolve()
        self._callback_group = ReentrantCallbackGroup()
        self._lock = Lock()
        self._bindings: dict[str, _BindingRuntime] = {}
        self._contexts: dict[tuple[int, int], _CallContext] = {}
        self._raw_servers: list[Any] = []
        self._typed_servers: list[Any] = []
        self._stop_requested = Event()
        self._poll_thread: Thread | None = None
        self._closed = False

        try:
            self._initialize_runtime()
        except BaseException:
            self._close_runtime()
            raise

    def _initialize_runtime(self) -> None:
        from hakoniwa_pdu_rpc import RpcMuxServer, make_typed_server

        resolved_by_name = {
            service.binding.hakoniwa_service: service
            for service in self._generated.services
        }
        expected = {binding.hakoniwa_service for binding in self._config.bindings}
        if set(resolved_by_name) != expected:
            raise ValueError("Resolved services do not match the Service Binding")

        bindings_by_node: dict[str, list[ServiceBinding]] = defaultdict(list)
        for binding in self._config.bindings:
            bindings_by_node[binding.server_endpoint.node_id].append(binding)

        for node_id, bindings in bindings_by_node.items():
            raw_server = RpcMuxServer(
                self._rpc_library,
                node_id,
                self._generated.server_config,
                _endpoint_config_for_node(self._generated, node_id),
                self._config.service.delta_time_usec,
                self._config.service.time_source_type,
            )
            raw_server.start()
            packages = {
                binding.hakoniwa_service: (
                    "hakoniwa_pdu.pdu_msgs."
                    + resolved_by_name[binding.hakoniwa_service]
                    .pdu_service_type.split("/", 1)[0]
                )
                for binding in bindings
            }
            typed_server = make_typed_server(
                raw_server,
                self._generated.server_config,
                packages=packages,
            )
            self._raw_servers.append(raw_server)
            self._typed_servers.append(typed_server)

            for binding in bindings:
                resolved = resolved_by_name[binding.hakoniwa_service]
                ros_service_type = import_ros_service_class(binding.ros_type)
                ros_client = self.create_client(
                    ros_service_type,
                    binding.ros_name,
                    callback_group=self._callback_group,
                )
                runtime = _BindingRuntime(
                    binding=binding,
                    resolved=resolved,
                    typed_service=typed_server.service(binding.hakoniwa_service),
                    ros_client=ros_client,
                    ros_service_type=ros_service_type,
                )
                self._bindings[binding.hakoniwa_service] = runtime
                self.get_logger().info(
                    "service client ready: "
                    f"ros_name={binding.ros_name} "
                    f"ros_type={binding.ros_type} "
                    f"rpc_service={binding.hakoniwa_service} "
                    f"pdu_type={resolved.pdu_service_type} "
                    f"max_clients={binding.max_clients} "
                    f"timeout_msec={binding.timeout_msec}"
                )

        self._poll_thread = Thread(
            target=self._poll_loop,
            name="hakoniwa-service-server-poll",
            daemon=True,
        )
        self._poll_thread.start()

    def _poll_loop(self) -> None:
        while not self._stop_requested.is_set():
            delivered = False
            for server_index, typed_server in enumerate(self._typed_servers):
                try:
                    event = typed_server.poll()
                except BaseException as error:
                    request = getattr(error, "request", None)
                    if request is not None:
                        self._reply_error(
                            request,
                            "INVALID",
                            f"request decode failed: {error}",
                        )
                    elif not self._stop_requested.is_set():
                        self.get_logger().error(
                            f"Hakoniwa Service Server poll failed: {error}"
                        )
                    continue
                if event.event.name == "NONE":
                    continue
                delivered = True
                if event.event.name == "REQUEST_IN":
                    self._handle_request(server_index, event)
                elif event.event.name == "REQUEST_CANCEL":
                    self._handle_cancel(server_index, event)
                else:
                    self.get_logger().error(
                        f"Unsupported Hakoniwa Service event: {event.event.name}"
                    )
            self._expire_calls()
            if not delivered:
                time.sleep(0.001)

    def _handle_request(self, server_index: int, request: Any) -> None:
        runtime = self._bindings.get(request.service_name)
        if runtime is None:
            self.get_logger().error(
                f"Hakoniwa request references unknown Service: {request.service_name}"
            )
            return
        key = (server_index, request.request_token)
        with self._lock:
            if key in self._contexts:
                self.get_logger().error(
                    "Duplicate Hakoniwa request token: "
                    f"service={request.service_name} token={request.request_token}"
                )
                return

        if not runtime.ros_client.service_is_ready():
            self._send_error(
                runtime,
                request,
                "NOT_SUPPORTED",
                f"ROS Service is unavailable: {runtime.binding.ros_name}",
            )
            return
        try:
            ros_request = runtime.ros_service_type.Request()
            copy_matching_fields(request.request_body, ros_request)
        except BaseException as error:
            self._send_error(
                runtime,
                request,
                "INVALID",
                f"request conversion failed: {error}",
            )
            return

        try:
            future = runtime.ros_client.call_async(ros_request)
            context = _CallContext(request, runtime, future, time.monotonic())
            with self._lock:
                self._contexts[key] = context
            future.add_done_callback(
                lambda completed: self._on_ros_response(
                    key,
                    context,
                    completed,
                )
            )
        except BaseException as error:
            self._send_error(
                runtime,
                request,
                "ERROR",
                f"ROS Service call failed: {error}",
            )

    def _on_ros_response(
        self,
        key: tuple[int, int],
        context: _CallContext,
        future: Any,
    ) -> None:
        with self._lock:
            if context.terminal or self._contexts.get(key) is not context:
                return
            try:
                ros_response = future.result()
                response = context.runtime.typed_service.create_response()
                copy_matching_fields(ros_response, response)
            except BaseException as error:
                self._send_error_locked(
                    context.runtime,
                    context.request,
                    "ERROR",
                    f"ROS response failed: {error}",
                )
            else:
                try:
                    context.runtime.typed_service.send_reply(
                        context.request,
                        response,
                    )
                except BaseException as error:
                    # Delivery may already have started. Do not attempt a
                    # second protocol response after a normal send failure.
                    self.get_logger().error(
                        "Hakoniwa response send failed: "
                        f"service={context.request.service_name} "
                        f"token={context.request.request_token} error={error}"
                    )
            context.terminal = True
            self._contexts.pop(key, None)

    def _handle_cancel(self, server_index: int, request: Any) -> None:
        key = (server_index, request.request_token)
        future = None
        with self._lock:
            context = self._contexts.get(key)
            if context is None or context.terminal:
                self.get_logger().error(
                    "Hakoniwa Cancel has no active ROS call: "
                    f"service={request.service_name} token={request.request_token}"
                )
                return
            if context.request.service_name != request.service_name:
                self.get_logger().error(
                    "Hakoniwa Cancel Service mismatch: "
                    f"expected={context.request.service_name} "
                    f"actual={request.service_name} token={request.request_token}"
                )
                return
            try:
                context.runtime.typed_service.send_cancel_reply(request)
            except BaseException as error:
                self.get_logger().error(
                    "Hakoniwa Cancel reply failed: "
                    f"service={request.service_name} "
                    f"token={request.request_token} error={error}"
                )
                return
            context.terminal = True
            self._contexts.pop(key, None)
            future = context.ros_future
        # rclpy may run completion callbacks synchronously from cancel(). Do
        # not call it while holding the context mutex.
        future.cancel()

    def _expire_calls(self) -> None:
        now = time.monotonic()
        futures = []
        with self._lock:
            expired = [
                (key, context)
                for key, context in self._contexts.items()
                if not context.terminal
                and (now - context.started_at) * 1000
                >= context.runtime.binding.timeout_msec
            ]
            for key, context in expired:
                self._send_error_locked(
                    context.runtime,
                    context.request,
                    "ERROR",
                    "ROS Service response timed out: "
                    f"{context.runtime.binding.ros_name}",
                )
                context.terminal = True
                self._contexts.pop(key, None)
                futures.append(context.ros_future)
        for future in futures:
            future.cancel()

    def _reply_error(
        self,
        request: Any,
        result_name: str,
        message: str,
    ) -> None:
        runtime = self._bindings.get(request.service_name)
        if runtime is None:
            self.get_logger().error(message)
            return
        self._send_error(runtime, request, result_name, message)

    def _send_error(
        self,
        runtime: _BindingRuntime,
        request: Any,
        result_name: str,
        message: str,
    ) -> None:
        with self._lock:
            self._send_error_locked(runtime, request, result_name, message)

    def _send_error_locked(
        self,
        runtime: _BindingRuntime,
        request: Any,
        result_name: str,
        message: str,
    ) -> None:
        self.get_logger().error(message)
        try:
            from hakoniwa_pdu_rpc import RpcServiceResultCode

            runtime.typed_service.send_error(
                request,
                getattr(RpcServiceResultCode, result_name),
            )
        except BaseException as error:
            self.get_logger().error(
                "Hakoniwa error reply failed: "
                f"service={request.service_name} "
                f"token={request.request_token} error={error}"
            )

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
            futures = []
            for context in self._contexts.values():
                context.terminal = True
                futures.append(context.ros_future)
            self._contexts.clear()
        for future in futures:
            future.cancel()
        for server in self._raw_servers:
            try:
                server.stop()
            except BaseException as error:
                self.get_logger().error(f"RPC Server stop failed: {error}")
            server.close()
        self._raw_servers.clear()
        self._typed_servers.clear()
        for runtime in self._bindings.values():
            self.destroy_client(runtime.ros_client)
        self._bindings.clear()


def _endpoint_config_for_node(
    generated: GeneratedServiceConfigs,
    node_id: str,
) -> Path:
    document = json.loads(generated.endpoint_config.read_text(encoding="utf-8"))
    for node in document:
        if node.get("nodeId") != node_id:
            continue
        endpoints = node.get("endpoints")
        if not isinstance(endpoints, list) or len(endpoints) != 1:
            break
        relative = endpoints[0].get("config_path")
        if isinstance(relative, str) and relative:
            path = (generated.output_dir / relative).resolve()
            if path.is_file():
                return path
    raise ValueError(f"Generated RPC endpoint was not found for node: {node_id}")


def run(
    config_path: str | Path,
    *,
    output_dir: str | Path | None = None,
    offset_dir: str | Path | None = None,
    rpc_library: str | Path | None = None,
) -> None:
    configure_import_paths()
    library = rpc_library or os.environ.get("HAKO_PDU_RPC_LIBRARY")
    if not library:
        raise ValueError("Specify --rpc-library or set HAKO_PDU_RPC_LIBRARY")
    generated = generate_service_configs(
        config_path,
        output_dir=output_dir,
        offset_dir=offset_dir,
    )
    config = load_service_binding(config_path)

    rclpy.init()
    node: HakoniwaRosServiceClientNode | None = None
    executor: MultiThreadedExecutor | None = None
    try:
        node = HakoniwaRosServiceClientNode(config, generated, library)
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
        description="Run the Hakoniwa Typed RPC Server to ROS Service Client bridge"
    )
    parser.add_argument("--config", required=True, help="Service Binding JSON path")
    parser.add_argument("--output-dir", help="Generated RPC config output directory")
    parser.add_argument("--offset-dir", help="Hakoniwa offset root")
    parser.add_argument("--rpc-library", help="PDU-RPC shared library path")
    args = parser.parse_args()
    try:
        run(
            args.config,
            output_dir=args.output_dir,
            offset_dir=args.offset_dir,
            rpc_library=args.rpc_library,
        )
    except ValueError as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
