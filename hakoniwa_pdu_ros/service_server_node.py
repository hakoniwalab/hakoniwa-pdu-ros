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
from hakoniwa_pdu_ros.service_config_generator import generate_service_configs
from hakoniwa_pdu_ros.type_mapper import copy_matching_fields, import_ros_service_class


class HakoniwaRosServiceServerNode(Node):
    """ROS Service Server backed by independent Typed Hakoniwa RPC clients."""

    def __init__(
        self,
        config: ServiceBindingConfig,
        rpc_service_config: str | Path,
        rpc_library: str | Path,
    ) -> None:
        super().__init__("hakoniwa_pdu_ros_service_server")
        self._config = config
        self._rpc_service_config = Path(rpc_service_config).resolve()
        self._rpc_library = Path(rpc_library).resolve()
        self._callback_group = ReentrantCallbackGroup()
        self._pools: dict[str, RpcClientPool] = {}
        self._services: list[Any] = []
        self._closed = False

        try:
            for binding in config.bindings:
                pool = self._create_pool(binding)
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
                    f"service ready: ros={binding.ros_name} "
                    f"rpc={binding.hakoniwa_service} max_clients={binding.max_clients}"
                )
        except BaseException:
            self._close_runtime()
            raise

    def _create_pool(self, binding: ServiceBinding) -> RpcClientPool:
        from hakoniwa_pdu_rpc import RpcClient, make_typed_client

        service_type = binding.ros_type.rsplit("/", 1)[-1]
        package = None
        if binding.pdu_service_type is not None:
            package_name, service_type = binding.pdu_service_type.split("/", 1)
            package = f"hakoniwa_pdu.pdu_msgs.{package_name}"

        def client_factory(name: str):
            rpc_client = RpcClient(
                self._rpc_library,
                self._config.rpc.client_endpoint.node_id,
                name,
                self._rpc_service_config,
                self._config.rpc.endpoint_config,
                self._config.rpc.delta_time_usec,
                self._config.rpc.time_source_type,
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
                raise RuntimeError(f"RPC client pool is busy: {binding.hakoniwa_service}")

            try:
                rpc_request = lease.typed_client.create_request()
                copy_matching_fields(request, rpc_request)
                rpc_future = lease.typed_client.call_async(
                    rpc_request,
                    timeout_usec=binding.timeout_msec * 1000,
                )
                pool.set_future(lease, rpc_future)

                completion = RclpyFuture(executor=self.executor)

                def complete(future) -> None:
                    try:
                        completion.set_result(future.result())
                    except BaseException as error:
                        completion.set_exception(error)

                rpc_future.add_done_callback(complete)
                rpc_response = await completion
                copy_matching_fields(rpc_response, response)
                return response
            except BaseException as error:
                self.get_logger().error(
                    f"service call failed: ros={binding.ros_name} "
                    f"rpc={binding.hakoniwa_service} error={error}"
                )
                raise
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
        node = HakoniwaRosServiceServerNode(config, generated.client_config, library)
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
