# Docker native integration tests

This directory runs `hakoniwa-pdu-ros` against a real ROS 2 runtime instead of the fake ROS modules used by the fast unit tests.

## Environment

- Ubuntu 24.04
- ROS 2 Jazzy
- `rmw_cyclonedds_cpp`
- real `rclpy` message classes
- installed `example_interfaces/srv/AddTwoInts` IDL
- PyPI `hakoniwa-pdu`
- a pinned, Core-free `hakoniwa-pdu-endpoint` shared-library build
- a pinned `hakoniwa-pdu-rpc` shared-library build with Typed `call_async()`
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

3. Service config generation
   - resolves the installed `example_interfaces/srv/AddTwoInts` class and `.srv`
   - resolves the PyPI `hakoniwa-pdu` generated service type
   - generates matching server/client RPC configs with deterministic client names and channels

4. Native AddTwoInts RPC baseline
   - starts the reusable test RPC server over the real TCP Endpoint transport
   - creates a Typed RPC client from the generated client config
   - calls `TypedRpcClient.call_async()` with `19 + 23`
   - verifies the server received the typed request and the client received `42`

This fourth group deliberately contains no ROS Service Node. It establishes the
Hakoniwa RPC baseline that the ROS Service Server Node tests will reuse next.

5. ROS Service Server Node E2E
   - starts the same reusable Hakoniwa AddTwoInts RPC server
   - starts `HakoniwaRosServiceServerNode` with four Typed RPC clients
   - calls `/add_two_ints` from a real `rclpy` Service Client
   - verifies the ROS request reaches RPC and the ROS response contains `42`

The Endpoint and RPC source revisions are pinned by Docker build arguments in
`Dockerfile`. Update those arguments intentionally when adopting a newer
dependency contract.

Every graph test has an explicit timeout and destroys all nodes and executors before returning.
