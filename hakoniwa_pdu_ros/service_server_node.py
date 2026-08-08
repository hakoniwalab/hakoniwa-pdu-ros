from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Any

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future as RclpyFuture

from hakoniwa_pdu_ros.env_setup import configure_import_paths
from hakoniwa_pdu_ros.service_binding import (
    ServiceBinding,
    ServiceBindingConfig,
    load_service_binding,
    service_key,
)
from hakoniwa_pdu_ros.service_client_pool import (
    RpcClientPool,
    create_rpc_client_pool,
)
from hakoniwa_pdu_ros.service_call_lifecycle import BridgeCallLifecycle
from hakoniwa_pdu_ros.service_config_generator import (
    ResolvedService,
    generate_service_configs,
)
from hakoniwa_pdu_ros.type_mapper import copy_matching_fields, import_ros_service_class


class HakoniwaRosServiceServerNode(Node):
    """ROS Service Server backed by independent Typed Hakoniwa RPC clients."""

    def __init__(
        self,
        config: ServiceBindingConfig,
        rpc_service_config: str | Path,
        rpc_endpoint_config: str | Path,
        rpc_library: str | Path,
        resolved_services: tuple[ResolvedService, ...],
    ) -> None:
        super().__init__("hakoniwa_pdu_ros_service_server")
        self._config = config
        self._rpc_service_config = Path(rpc_service_config).resolve()
        self._rpc_endpoint_config = Path(rpc_endpoint_config).resolve()
        self._rpc_library = Path(rpc_library).resolve()
        self._callback_group = ReentrantCallbackGroup()
        self._pools: dict[str, RpcClientPool] = {}
        self._services: list[Any] = []
        self._closed = False

        resolved_by_service = {
            service.binding.hakoniwa_service: service
            for service in resolved_services
        }
        expected_services = {binding.hakoniwa_service for binding in config.bindings}
        if set(resolved_by_service) != expected_services:
            raise ValueError("Resolved services do not match the Service Binding")

        try:
            for binding in config.bindings:
                resolved = resolved_by_service[binding.hakoniwa_service]
                pool = self._create_pool(binding, resolved.pdu_service_type)
                self._pools[binding.hakoniwa_service] = pool
                service_class = import_ros_service_class(binding.ros_type)
                service = self.create_service(
                    service_class,
                    binding.ros_name,
                    self._make_callback(binding, pool),
                    callback_group=self._callback_group,
                )
                self._services.append(service)
                self.get_logger().info(
                    f"service ready: ros_name={binding.ros_name} "
                    f"ros_type={binding.ros_type} "
                    f"rpc_service={binding.hakoniwa_service} "
                    f"pdu_type={resolved.pdu_service_type} "
                    f"clients={_client_range(binding)} "
                    f"timeout_msec={binding.timeout_msec}"
                )
        except BaseException:
            self._close_runtime()
            raise

    def _create_pool(
        self, binding: ServiceBinding, pdu_service_type: str
    ) -> RpcClientPool:
        from hakoniwa_pdu_rpc import RpcClient, make_typed_client

        package_name, service_type = pdu_service_type.split("/", 1)
        package = f"hakoniwa_pdu.pdu_msgs.{package_name}"

        def client_factory(name: str):
            rpc_client = RpcClient(
                self._rpc_library,
                binding.client_endpoint.node_id,
                name,
                self._rpc_service_config,
                self._rpc_endpoint_config,
                self._config.service.delta_time_usec,
                self._config.service.time_source_type,
            )
            typed_client = make_typed_client(
                rpc_client,
                binding.hakoniwa_service,
                service_type,
                package=package,
            )
            return rpc_client, typed_client

        key = service_key(binding.hakoniwa_service)
        return create_rpc_client_pool(
            max_clients=binding.max_clients,
            client_name=lambda index: f"hakoniwa_pdu_ros_{key}_{index}",
            client_factory=client_factory,
        )

    def _make_callback(self, binding: ServiceBinding, pool: RpcClientPool):
        async def callback(request: object, response: object) -> object:
            lease = pool.acquire()
            if lease is None:
                self.get_logger().error(
                    "BUSY "
                    f"ros_service={binding.ros_name} "
                    f"rpc_service={binding.hakoniwa_service} "
                    f"active_clients={pool.active_count} max_clients={pool.capacity}"
                )
                raise ServiceCallRejected(
                    f"RPC client pool is busy: {binding.hakoniwa_service}"
                )

            try:
                rpc_request = lease.typed_client.create_request()
                _copy_service_fields(
                    request,
                    rpc_request,
                    direction="request",
                    binding=binding,
                )
                rpc_future = lease.typed_client.call_async(
                    rpc_request,
                    # The bridge lifecycle below is the sole deadline owner.
                    # A native timeout here would race its protocol cancel
                    # against BridgeCallLifecycle._expire().  PDU-RPC defines
                    # zero as an infinite wait, so terminal cleanup remains
                    # owned by the RPC future after the bridge requests cancel.
                    timeout_usec=0,
                )
                pool.set_future(lease, rpc_future)

                completion = RclpyFuture(executor=self.executor)

                BridgeCallLifecycle(
                    rpc_future,
                    timeout_msec=binding.timeout_msec,
                    on_result=completion.set_result,
                    on_error=completion.set_exception,
                )
                rpc_response = await completion
                _copy_service_fields(
                    rpc_response,
                    response,
                    direction="response",
                    binding=binding,
                )
                return response
            except ServiceConversionError as error:
                self.get_logger().error(str(error))
                raise ServiceCallRejected(str(error)) from error
            except BaseException as error:
                self.get_logger().error(
                    f"service call failed: ros={binding.ros_name} "
                    f"rpc={binding.hakoniwa_service} error={error}"
                )
                raise ServiceCallRejected(str(error)) from error
            finally:
                pool.release(lease)

        return callback

    def destroy_node(self) -> bool:
        self._close_runtime()
        return super().destroy_node()

    def _close_runtime(self) -> None:
        if self._closed:
            return
        self._closed = True
        for service in self._services:
            self.destroy_service(service)
        self._services.clear()
        for pool in self._pools.values():
            pool.close()
        self._pools.clear()


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
    node: HakoniwaRosServiceServerNode | None = None
    executor: MultiThreadedExecutor | None = None
    try:
        node = HakoniwaRosServiceServerNode(
            config,
            generated.client_config,
            generated.endpoint_config,
            library,
            generated.services,
        )
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        _spin_service_executor(executor)
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
        description="Run the ROS Service Server to Hakoniwa Typed RPC Client bridge"
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


def _client_range(binding: ServiceBinding) -> str:
    key = service_key(binding.hakoniwa_service)
    first = f"hakoniwa_pdu_ros_{key}_0"
    if binding.max_clients == 1:
        return first
    return f"{first}..hakoniwa_pdu_ros_{key}_{binding.max_clients - 1}"


class ServiceCallRejected(RuntimeError):
    """One ROS request intentionally receives no synthesized response."""


class ServiceConversionError(RuntimeError):
    """Request or response conversion failed at the ROS/RPC boundary."""

    def __init__(
        self,
        *,
        direction: str,
        binding: ServiceBinding,
        cause: BaseException,
    ) -> None:
        self.direction = direction
        self.ros_service = binding.ros_name
        self.rpc_service = binding.hakoniwa_service
        self.cause = cause
        super().__init__(
            "service conversion failed: "
            f"direction={direction} "
            f"ros_service={binding.ros_name} "
            f"rpc_service={binding.hakoniwa_service} "
            f"error={cause}"
        )


def _copy_service_fields(
    source: object,
    target: object,
    *,
    direction: str,
    binding: ServiceBinding,
) -> None:
    try:
        copy_matching_fields(source, target)
    except BaseException as error:
        raise ServiceConversionError(
            direction=direction,
            binding=binding,
            cause=error,
        ) from error


def _spin_service_executor(executor: MultiThreadedExecutor) -> None:
    """Keep the bridge alive when an individual Service call is rejected."""
    while rclpy.ok():
        try:
            executor.spin()
            return
        except ServiceCallRejected:
            continue
