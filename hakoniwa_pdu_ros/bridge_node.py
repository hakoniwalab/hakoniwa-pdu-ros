import rclpy
from rclpy.node import Node

from hakoniwa_pdu_ros.config_loader import BindingConfig, BindingRootConfig, load_config
from hakoniwa_pdu_ros.pdu_endpoint import PduEndpointManager
from hakoniwa_pdu_ros.type_mapper import (
    import_ros_msg_class,
    pdu_bytes_to_ros_msg,
    ros_msg_to_pdu_bytes,
    validate_pdu_converter,
)
from hakoniwa_pdu_ros.zenoh_io import validate_zenoh_io_for_config

class HakoniwaRosBridgeNode(Node):
    """ROS 2 bridge node for Hakoniwa PDU bindings."""

    def __init__(self, config: BindingRootConfig | None = None) -> None:
        super().__init__("hakoniwa_pdu_ros_bridge")
        self._config = config
        self._manager: PduEndpointManager | None = None
        self._publishers = []
        self._subscriptions = []
        self.get_logger().info("hakoniwa_pdu_ros bridge node started")
        if self._config is not None:
            self._manager = PduEndpointManager(self._config.endpoint_config)
            self._manager.start()
            self.get_logger().info(
                f"loaded {len(self._config.bindings)} bindings from {self._config.endpoint_config}"
            )
            for binding in self._config.bindings:
                validate_pdu_converter(binding.pdu_type)
                if binding.direction == "pdu_to_ros":
                    self._setup_in_binding(binding)
                else:
                    self._setup_out_binding(binding)

    def destroy_node(self) -> bool:
        if self._manager is not None:
            self._manager.stop()
            self._manager = None
        return super().destroy_node()

    def _setup_in_binding(self, binding: BindingConfig) -> None:
        msg_cls = import_ros_msg_class(binding.pdu_type)
        publisher = self.create_publisher(msg_cls, binding.topic, 10)
        self._publishers.append(publisher)

        def _on_recv(data: bytes) -> None:
            msg = pdu_bytes_to_ros_msg(data, binding.pdu_type)
            publisher.publish(msg)

        assert self._manager is not None
        self._manager.subscribe_recv(
            binding.pdu_key.robot_name,
            binding.pdu_key.pdu_name,
            _on_recv,
        )

    def _setup_out_binding(self, binding: BindingConfig) -> None:
        msg_cls = import_ros_msg_class(binding.pdu_type)

        def _on_msg(msg: object) -> None:
            data = ros_msg_to_pdu_bytes(msg, binding.pdu_type)
            assert self._manager is not None
            self._manager.send(
                binding.pdu_key.robot_name,
                binding.pdu_key.pdu_name,
                data,
            )

        subscription = self.create_subscription(msg_cls, binding.topic, _on_msg, 10)
        self._subscriptions.append(subscription)


def run(config_path: str | None = None) -> None:
    if config_path:
        validate_zenoh_io_for_config(config_path)
    rclpy.init()
    config = load_config(config_path) if config_path else None
    node = HakoniwaRosBridgeNode(config)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
