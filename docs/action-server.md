# ROS 2 Action Server Bridge

## Scope

`hakoniwa-pdu-ros` exposes a ROS 2 Action Server backed by a Hakoniwa Action Client.
Application behavior stays in the Hakoniwa Action Server.

```text
ROS 2 Action Client
        |
        | Goal / Cancel
        v
hakoniwa-pdu-ros Action Server
        |
        | Hakoniwa Goal / Cancel
        v
hakoniwa-pdu-rpc Action Client
        |
        v
Hakoniwa Action Server

Hakoniwa Feedback / Result
        ^
        |
hakoniwa-pdu-ros
        |
        v
ROS 2 Feedback / Result
```

The reverse direction, where a Hakoniwa Asset calls a ROS 2 Action Server, is outside this contract.

## Responsibility boundary

The bridge owns only ROS/Hakoniwa adaptation:

- load the configured ROS Action type;
- map ROS Goal fields into the generated Hakoniwa Action request body;
- preserve the ROS Goal UUID as the Hakoniwa `goal_id`;
- map Hakoniwa Goal acceptance/rejection to ROS Goal acceptance/rejection;
- publish Hakoniwa Feedback as ROS Feedback;
- forward ROS Cancel to Hakoniwa Cancel;
- map Hakoniwa terminal status and Result into ROS terminal state and Result.

The bridge does not reimplement the Hakoniwa Action state machine. Goal ownership, slot capacity, Cancel/Result races, feedback sequencing, and terminal-state validation remain in `hakoniwa-pdu-rpc`.

## Goal acceptance and identity

The existing Action PDU design requires the ROS Goal UUID to pass through unchanged as the Hakoniwa 128-bit `goal_id`.

```text
ROS Goal UUID == Hakoniwa goal_id
```

This keeps one execution identity across the ROS and Hakoniwa boundaries. Feedback, Cancel, and Result therefore use the same Goal identity that originated at the ROS Action Client.

The public Jazzy `rclpy.ActionServer` goal callback receives only the user-defined Goal payload. Internally, however, `ActionServer._execute_goal_request()` has already taken the full SendGoal request and obtained `goal_request.goal_id` before invoking that callback.

To preserve the identity contract without reimplementing the ROS Action lifecycle, `hakoniwa-pdu-ros` uses a narrow `ActionServer` subclass. The override captures the ROS Goal UUID in a context-local value and immediately delegates the complete request processing to the upstream rclpy implementation. The bridge goal callback reads that UUID and sends the Hakoniwa Goal with the exact same 16 bytes.

The acceptance sequence is:

```text
ROS SendGoal request
    -> capture ROS Goal UUID
    -> encode Hakoniwa Goal PDU
    -> send Hakoniwa Goal with the same 128-bit goal_id
    -> wait for Hakoniwa GOAL_RESPONSE

Hakoniwa ACCEPTED
    -> return ROS GoalResponse.ACCEPT
    -> execute the ROS Goal

Hakoniwa REJECTED / timeout / transport error
    -> return ROS GoalResponse.REJECT
```

This preserves both important contracts:

1. ROS Goal rejection remains rejection, not a later `ABORTED` result.
2. ROS Goal UUID and Hakoniwa `goal_id` remain identical end to end.

The rclpy hook is intentionally limited to exposing information that rclpy already holds before the user callback. Goal tracking, response transmission, status publication, cancellation processing, result handling, and goal expiration remain owned by the upstream `ActionServer` implementation.

## Event polling

One background pump is the sole caller of `hakoniwa_pdu_rpc.ActionClient.poll()`.
It dispatches events to Goal-local inboxes by Goal ID.

Goal-local inboxes support event-type matching without removing unrelated events. This matters because the ROS execute callback and Cancel callback can wait concurrently:

- execute waits for `FEEDBACK`, `RESULT`, `TIMEOUT`, or `ERROR`;
- cancel waits for `CANCEL_RESPONSE`, `RESULT`, or `ERROR`.

A Cancel waiter therefore cannot consume and lose Feedback or Result intended for the execute path.

## Status mapping

| Hakoniwa terminal status | ROS Goal terminal operation |
| --- | --- |
| `SUCCEEDED` | `goal_handle.succeed()` |
| `CANCELED` | `goal_handle.canceled()` |
| `ABORTED` | `goal_handle.abort()` |

Hakoniwa Goal rejection maps to ROS Goal rejection before a ROS Goal Handle is accepted.

## Binding configuration

Example:

```json
{
  "version": 1,
  "runtime": {
    "node_id": "ros-action-bridge",
    "client_name": "ros-action-client",
    "action_config": "resolved-action.json",
    "endpoint_config": "endpoints.json",
    "delta_time_usec": 1000,
    "time_source_type": "real"
  },
  "actions": [
    {
      "ros_name": "/fibonacci",
      "ros_type": "example_interfaces/action/Fibonacci",
      "hakoniwa_action": "fibonacci",
      "pdu_action_type": "sample_action_msgs/Fibonacci",
      "goal_response_timeout_msec": 5000
    }
  ]
}
```

`pdu_action_type` follows the generated PDU naming convention:

```text
<package>/<Type>
  -> <Type>ActionRequest
  -> <Type>ActionFeedback
  -> <Type>ActionResponse
```

Paths in `runtime.action_config` and `runtime.endpoint_config` are resolved relative to the Action Binding file when they are not absolute.

## Run

```bash
export HAKO_PDU_RPC_LIBRARY=/path/to/libhakoniwa_pdu_rpc.so
ros2 run hakoniwa_pdu_ros action-server --config /path/to/action-binding.json
```

or:

```bash
ros2 run hakoniwa_pdu_ros action-server \
  --config /path/to/action-binding.json \
  --rpc-library /path/to/libhakoniwa_pdu_rpc.so
```

## Validation boundary

Repository tests cover:

- Action Binding validation and path resolution;
- independent event routing for multiple Hakoniwa Goals;
- Cancel/execute concurrent waits without event loss;
- use of the merged `hakoniwa-pdu-rpc` Action Python API in the ROS 2 native test image.

A full ROS 2 Action Client -> bridge -> real Hakoniwa Fibonacci Action Server end-to-end fixture should be added before declaring native Action behavior fully verified. This document and implementation do not claim that final E2E evidence yet.
