# Docker native integration tests

This directory runs `hakoniwa-pdu-ros` against a real ROS 2 runtime instead of the fake ROS modules used by the fast unit tests.

## Environment

- Ubuntu 24.04
- ROS 2 Jazzy
- `rmw_cyclonedds_cpp`
- real `rclpy` message classes
- installed `example_interfaces/srv/AddTwoInts` IDL
- installed `action_tutorials_interfaces/action/Fibonacci` IDL
- PyPI `hakoniwa-pdu`
- a pinned, Core-free `hakoniwa-pdu-endpoint` shared-library build
- a pinned `hakoniwa-pdu-rpc` shared-library build with Typed RPC and Action APIs
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
   - starts the reusable test RPC server over the real `tcp_mux` Endpoint transport
   - creates a Typed RPC client from the generated client config
   - calls `TypedRpcClient.call_async()` with `19 + 23`
   - verifies the server received the typed request and the client received `42`

This fourth group deliberately contains no ROS Service Node. It establishes the
Hakoniwa RPC baseline reused by the ROS Service Server Node tests.

5. ROS Service Server Node E2E
   - starts the same reusable Hakoniwa AddTwoInts RPC server
   - starts `HakoniwaRosServiceServerNode` with four Typed RPC clients
   - calls `/add_two_ints` from a real `rclpy` Service Client
   - verifies the ROS request reaches RPC and the ROS response contains `42`
   - verifies two consecutive calls reuse a connected RPC client
   - verifies four parallel calls complete through four independent RPC clients
   - verifies a fifth request is rejected as `BUSY` while all clients are occupied
   - verifies a normal response arriving after the Bridge timeout is discarded
   - verifies the RPC client is reusable after terminal timeout cleanup
   - verifies shutdown during an active call sends protocol cancellation, waits
     for terminal cleanup, closes the pool, and synthesizes no ROS response
   - injects request and response conversion failures, verifies directional
     diagnostics, no synthesized response, lease release, and a later successful call

6. ROS Action Server Node E2E
   - generates the Hakoniwa Action runtime configuration from
     `config/action/fibonacci.json`
   - starts a real Hakoniwa Fibonacci Action Server over the CFFI/TCP runtime
   - starts `HakoniwaRosActionServerNode`
   - sends a Goal from a real ROS 2 `ActionClient`
   - verifies Goal acceptance, ordered Feedback, a `SUCCEEDED` terminal status,
     and the Fibonacci Result body
   - verifies explicit Goal rejection and two consecutive Goals
   - verifies accepted and rejected Cancel requests
   - verifies a normal Result winning while Cancel is pending
   - keeps four Goals active and verifies the fifth is rejected when all slots
     are occupied
   - verifies a missing Hakoniwa Goal Response becomes ROS Goal rejection at
     the configured timeout

7. ROS Action Client Node E2E
   - starts a real ROS 2 Fibonacci Action Server
   - starts `HakoniwaRosActionClientNode` with the generated shared Action
     configuration
   - sends Goals from a real Hakoniwa Typed Action Client over TCP
   - verifies Goal acceptance and rejection, ordered Feedback, and a
     `SUCCEEDED` or `ABORTED` Result body
   - verifies accepted Cancel followed by a `CANCELED` Result
   - verifies rejected Cancel followed by normal Goal completion

The canonical ROS type is
`action_tutorials_interfaces/action/Fibonacci`. Its Feedback field
`partial_sequence` matches the generated
`sample_action_msgs/Fibonacci` PDU contract. The similarly named
`example_interfaces/action/Fibonacci` uses a different Feedback field and is
therefore not interchangeable.

For the equivalent user-observable three-process walkthrough, see
[`examples/service/README.md`](../../examples/service/README.md).

The fast unit suite covers the pool's `max_clients` capacity and `BUSY`
behavior. The native E2E uses the pinned PDU-RPC Python `RpcMuxServer` to cover
the same behavior with real parallel TCP connections.

The Endpoint and RPC source revisions are pinned by Docker build arguments in
`Dockerfile`. Update those arguments intentionally when adopting a newer
dependency contract. The RPC build is explicitly limited to two parallel jobs
so the Action-enabled build remains reproducible on Docker Desktop ARM hosts.

Until `sample_action_msgs` is included in a released `hakoniwa-pdu` wheel, the
Action E2E resolves that generated package from the pinned RPC repository's
Registry submodule. This is a test-environment bridge, not the final
user-install contract; remove it after the wheel contains the Action package.

Every graph test has an explicit timeout and destroys all nodes and executors before returning.
