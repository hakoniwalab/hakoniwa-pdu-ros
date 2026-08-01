# Hakoniwa Server Specification

## Purpose

This document defines the server-side contract for exposing Hakoniwa PDU-RPC
services to ROS 2 through `hakoniwa-pdu-ros`.

The supported direction is:

```text
ROS 2 Service Client
        |
        | ROS 2 Service request
        v
hakoniwa-pdu-ros
ROS 2 Service Server / Hakoniwa RPC Client
        |
        | PDU-RPC request
        v
Hakoniwa PDU-RPC Service Server
```

The reverse direction, where Hakoniwa calls a ROS 2 Service Server, is outside
the scope of this specification and must be defined separately.

## Design Principles

The server bridge follows these principles:

1. Reuse the existing `hakoniwa-pdu-rpc` request, timeout, cancellation, and
   response state machine.
2. Reuse `hakoniwa-pdu-endpoint` for transport and connection lifecycle.
3. Do not invent a bridge-specific service error payload.
4. Return a ROS 2 response only when a valid Hakoniwa RPC response is obtained.
5. Treat timeout and capacity limits as bridge-side resource-management events.
6. Make ROS 2 client timeout and failure interpretation the responsibility of
   the ROS 2 client application.

## Binding Configuration

The ROS-to-Hakoniwa mapping is declared separately from the PDU-RPC transport
configuration.

Example:

```json
{
  "ros_name": "/calculator/add",
  "ros_type": "example_interfaces/srv/AddTwoInts",
  "hakoniwa_service": "Service/Add",
  "timeout_msec": 3000
}
```

Fields:

| Field | Required | Meaning |
| --- | --- | --- |
| `ros_name` | yes | ROS 2 service name exposed by the bridge |
| `ros_type` | yes | ROS 2 service type |
| `hakoniwa_service` | yes | PDU-RPC service name |
| `timeout_msec` | yes | Bridge-side timeout for the Hakoniwa RPC operation |

The binding does not declare a fixed PDU-RPC client name. Client-side RPC
resources are allocated dynamically for each accepted ROS request.

The PDU-RPC configuration remains the source of truth for:

- `maxClients`
- request and response channels
- endpoint configuration
- transport configuration
- service PDU sizes and types

## Request Lifecycle

Each accepted ROS 2 Service request owns one temporary Hakoniwa RPC client
session.

```text
ROS request received
    -> allocate temporary RPC client resources
    -> start Endpoint connection
    -> confirm Endpoint running state
    -> send PDU-RPC request
    -> poll for RPC result
    -> normal response, timeout, disconnect, or send failure
    -> release Endpoint and RPC resources
```

A successful request follows this sequence:

```text
ROS request
    -> Endpoint connected
    -> PDU-RPC call
    -> RESPONSE_IN
    -> convert PDU response to ROS response
    -> return ROS response
    -> close temporary session
```

The temporary session is not reused for another ROS request.

This request-scoped lifecycle prevents a delayed response from one request from
being confused with a later request and makes connection cleanup the resource
release boundary.

## Concurrency and `maxClients`

`maxClients` means the maximum number of concurrent in-flight ROS Service
requests that can be backed by the Hakoniwa PDU-RPC service.

When the configured capacity is exhausted:

1. The Hakoniwa-side TCP connection is rejected or closed by the server-side
   connection manager.
2. The bridge detects failure through Endpoint running state, send failure, or
   the Endpoint disconnected callback.
3. The bridge does not start or continue the RPC operation.
4. The bridge releases all local resources.
5. The bridge writes an error log.
6. The bridge does not return a ROS 2 Service response.

The bridge does not queue a request while waiting for a free PDU-RPC client
slot. Capacity rejection is immediate from the bridge's point of view.

## Endpoint Connection Detection

The bridge uses the public `hakoniwa-pdu-endpoint` lifecycle interfaces.

Connection establishment is observed through Endpoint running state.

Disconnection is observed through the Endpoint disconnected callback, which
provides:

- endpoint name
- reason code
- reason text

A request is aborted immediately when a transport disconnection is detected.
No cancellation completion is required when the peer connection no longer
exists.

## Timeout Contract

`timeout_msec` is a bridge-side resource protection setting. It is not a ROS 2
Service cancellation protocol.

The timeout starts when the bridge begins processing the accepted ROS request.

`hakoniwa-pdu-rpc` defines timeout as a notification while the request remains
active:

```text
RUNNING
    -> RESPONSE_TIMEOUT
RUNNING
    -> explicit send_cancel_request()
CANCELLING
    -> RESPONSE_IN or RESPONSE_CANCEL
IDLE
```

Therefore, when the bridge observes `RESPONSE_TIMEOUT`, it must:

1. Mark the ROS request as timed out.
2. Explicitly send the PDU-RPC cancellation request.
3. Continue polling until either normal completion or cancellation completion
   resolves the PDU-RPC state, unless the transport disconnects first.
4. Discard any terminal response because the bridge timeout has already
   expired.
5. Close the temporary session and release its resources.
6. Write a timeout log.
7. Do not return a ROS 2 Service response.

A normal response may win the race after cancellation has been requested. That
response is still used to terminate the internal PDU-RPC state safely, but it
is not forwarded to ROS after the bridge timeout.

## ROS 2 Client Responsibility

ROS 2 Services do not provide a generic cancellation contract or a generic
transport-level error response.

The ROS 2 client application is responsible for:

- choosing its own response wait timeout
- stopping its local wait when that timeout expires
- deciding whether to retry
- interpreting missing responses as timeout, overload, transport failure, or
  application failure according to its own policy

The bridge does not synthesize a successful response, a zero-filled response,
or an application-specific error response for infrastructure failures.

## Result Handling

### Normal response

When `RESPONSE_IN` is received before the bridge timeout:

1. Validate the returned service and PDU response.
2. Convert the PDU response body into the configured ROS response type.
3. Return the ROS response.
4. Close the temporary session.

### PDU-RPC timeout

When `RESPONSE_TIMEOUT` is received:

1. Send an explicit cancel request.
2. Resolve the internal RPC state as described in the timeout contract.
3. Discard the final result.
4. Return no ROS response.

### Connection or send failure

When Endpoint connection establishment, request send, or transport operation
fails:

1. Do not send a PDU-RPC cancellation request when no live peer exists.
2. Close the temporary session immediately.
3. Return no ROS response.
4. Log the failure reason.

### Capacity rejection

When `maxClients` prevents the temporary Endpoint from remaining connected:

1. Treat the request as rejected.
2. Close local resources.
3. Return no ROS response.
4. Log current service and capacity context when available.

## Type Mapping

ROS Service Request and Response values are mapped to the corresponding
Hakoniwa generated PDU service types.

The mapping must follow the same runtime principles already used by the Topic
bridge:

- recursive field-name mapping
- generated PDU converters remain responsible for binary layout
- bridge code does not implement service PDU layouts manually
- unsupported or incompatible field mappings fail explicitly

Configuration and type compatibility should be validated during bridge startup
where possible. Runtime requests must not rely on silent numeric or structural
coercion.

## Observability

The first implementation requires structured logs for:

- service request accepted
- RPC connection failure
- RPC send failure
- `maxClients` rejection
- RPC timeout
- cancellation requested
- normal response winning the cancellation race
- cancellation completion
- transport disconnection
- response conversion failure

A later implementation may publish operational status through a ROS topic,
for example:

```text
/hakoniwa_pdu_ros/service_status
```

Potential status fields include:

- service name
- active clients
- maximum clients
- completed request count
- timeout count
- rejected request count
- transport error count
- last error

Status-topic publication is optional and is not required for the initial
server implementation.

## Required Runtime Adapter

`hakoniwa-pdu-ros` is implemented in Python, while the current
`hakoniwa-pdu-rpc` runtime client API is implemented in C++.

The server implementation therefore requires a thin Python-accessible adapter
for the PDU-RPC client lifecycle.

The adapter only needs to expose the client-side capabilities required by this
specification:

- create and destroy a temporary client session
- initialize and start Endpoint resources
- inspect Endpoint running state
- register a disconnected callback
- call a configured RPC service with request bytes
- poll RPC events
- send an explicit cancel request
- stop and close resources

The adapter should keep PDU-RPC state-machine details in the C++ layer and
expose a small stable boundary to Python. A C ABI with CFFI or an equivalent
thin binding is acceptable. This document does not mandate the binding
technology.

## Non-Goals

The initial server implementation does not provide:

- Hakoniwa-to-ROS Service calls
- ROS Action bridging
- queued waiting for a free `maxClients` slot
- automatic client retry
- generic ROS Service error responses
- persistent RPC client pooling
- bridge-specific request correlation independent of PDU-RPC
- guaranteed cancellation of arbitrary server-side application work

## Summary Contract

```text
Normal:
ROS request
    -> temporary Endpoint/RPC session
    -> PDU-RPC request
    -> valid PDU-RPC response
    -> ROS response
    -> release session

Timeout:
ROS request
    -> PDU-RPC timeout
    -> explicit PDU-RPC cancel
    -> internal terminal response
    -> discard result
    -> release session
    -> no ROS response

Capacity or transport failure:
ROS request
    -> connection/send/disconnect failure
    -> release session
    -> log error
    -> no ROS response
```
