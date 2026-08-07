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
- map Hakoniwa Goal acceptance/rejection to ROS Goal acceptance/rejection;
- publish Hakoniwa Feedback as ROS Feedback;
- forward ROS Cancel to Hakoniwa Cancel;
- map Hakoniwa terminal status and Result into ROS terminal state and Result;
- associate the ROS Goal UUID with the bridge-created Hakoniwa Goal ID while the Goal is active.

The bridge does not reimplement the Hakoniwa Action state machine. Goal ownership, slot capacity, Cancel/Result races, feedback sequencing, and terminal-state validation remain in `hakoniwa-pdu-rpc`.

## Goal acceptance

The ROS Goal callback does not receive a ROS Goal UUID through its public callback argument; the accepted `ServerGoalHandle` exposes the UUID later. The bridge therefore creates its own non-zero 128-bit Hakoniwa Goal ID for the Hakoniwa Goal handshake.

The acceptance sequence is:

```text
ROS Goal request
    -> encode Hakoniwa Goal PDU
    -> create Hakoniwa Goal ID
    -> send Hakoniwa Goal
    -> wait for Hakoniwa GOAL_RESPONSE

Hakoniwa ACCEPTED
    -> return ROS GoalResponse.ACCEPT
    -> associate ROS Goal UUID with Hakoniwa Goal session
    -> execute ROS Goal

Hakoniwa REJECTED / timeout / transport error
    -> return ROS GoalResponse.REJECT
```

This preserves the distinction between Goal rejection and execution abort. The bridge does not accept a ROS Goal first and later translate a Hakoniwa rejection into `ABORTED`.

## Goal identity

ROS and Hakoniwa Goal IDs are both 128 bit, but they are separate identities in this bridge implementation.

```text
ROS Goal UUID <-> bridge session <-> Hakoniwa Goal ID
```

The mapping exists only while the Goal is active and is released after the terminal Result.

## Event polling

One background pump is the sole caller of `hakoniwa_pdu_rpc.ActionClient.poll()`.
It dispatches events to Goal-local inboxes by Hakoniwa Goal ID.

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
