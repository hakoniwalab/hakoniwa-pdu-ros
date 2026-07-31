# Docker native integration tests

This directory runs `hakoniwa-pdu-ros` against a real ROS 2 runtime instead of the fake ROS modules used by the fast unit tests.

## Environment

- Ubuntu 24.04
- ROS 2 Jazzy
- `rmw_cyclonedds_cpp`
- real `rclpy` message classes
- PyPI `hakoniwa-pdu`
- the current `hakoniwa-pdu-ros` checkout installed in editable mode

Tobas itself is intentionally not part of the test dependency. The QoS tests publish the same `sensor_msgs/msg/JointState` shape and use the same `BEST_EFFORT` reliability that exposed the Tobas integration failure.

## Run locally

From the repository root:

```bash
bash test/docker/run_native_tests.sh
```

To isolate the test from another ROS graph, override the domain ID:

```bash
ROS_DOMAIN_ID=174 bash test/docker/run_native_tests.sh
```

## Coverage

The container runs two groups of tests:

1. Native message mapping
   - confirms that real `JointState`, `LaserScan`, and `Float64MultiArray` primitive sequences are represented by `array.array`
   - verifies ROS message -> PDU binary -> ROS message value equality

2. Native QoS graph behavior
   - `BEST_EFFORT` publisher -> `BEST_EFFORT` bridge subscription succeeds without a relay
   - `BEST_EFFORT` publisher -> `RELIABLE` bridge subscription does not deliver data and produces an incompatible QoS event
   - `RELIABLE` publisher -> binding with default QoS remains compatible

Every graph test has an explicit timeout and destroys all nodes and executors before returning.
