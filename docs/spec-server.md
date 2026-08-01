# ROS 2 Service Bridge Specification

[日本語](spec-server.ja.md)

## Purpose

This document defines the ROS 2 Service Server to Hakoniwa PDU-RPC Client
bridge contract.

```text
ROS 2 Service Client
        |
        v
HakoniwaRosServiceServerNode   (ROS Service Server)
        |
        v
hakoniwa-pdu-rpc RpcClient     (Hakoniwa RPC Client)
        |
        v
Hakoniwa Asset RPC Server
```

Topic bridging remains in the existing independent Topic Bridge Node and
configuration. The initial implementation does not cover the reverse RPC
direction, ROS Actions, or application-specific error payloads.

## Node Roles and Naming

`server` and `client` always describe the ROS-facing role. The current target is
`HakoniwaRosServiceServerNode`: it receives requests as a ROS Service Server and
acts as a Hakoniwa RPC Client.

The reverse direction will be implemented separately as
`HakoniwaRosServiceClientNode`. It will act as a ROS Service Client and a
Hakoniwa RPC Server. The two directions are not mixed in one node or Binding.

| Binding `kind` | ROS role | Hakoniwa role | Status |
| --- | --- | --- | --- |
| `ros_service_server` | Service Server | RPC Client | Defined by this specification |
| `ros_service_client` | Service Client | RPC Server | Future separate node and Binding contract |

The `server` and `client` in the generated filenames
`rpc-server-services.json` and `rpc-client-services.json` describe PDU-RPC
roles, not the Binding `kind`. A current `ros_service_server` Binding generates
both files so the Bridge and its static counterpart use the same resolved
configuration.

## Canonical Service Binding

The standalone Service Node reads a Service Binding conforming to
[`schema/service-binding.schema.json`](../schema/service-binding.schema.json).
An AddTwoInts example is available at
[`config/service/add_two_ints.json`](../config/service/add_two_ints.json).

The user declares:

- `kind`, identifying the ROS-facing role (currently `ros_service_server`);
- ROS service name and `package/srv/Type`;
- Hakoniwa service name;
- RPC client and server endpoint references;
- per-service `max_clients`;
- bridge timeout;
- optional request/response heap capacity;
- an optional PDU service type override.

The user does not declare RPC client names or channel IDs.

Relative `rpc.endpoint_config` paths are resolved from the Service Binding
file. Endpoint references must exist in that endpoint registry. Unknown fields,
duplicate service names, duplicate normalized service keys, invalid capacities,
and unresolved types are rejected before generation or startup.

## Config Generation

Generate role-specific PDU-RPC service configs with:

```bash
python3 -m hakoniwa_pdu_ros.generate_service_config \
  --config config/service/add_two_ints.json \
  --offset-dir /path/to/share/hakoniwa/offset
```

The generator resolves:

1. the installed ROS service class and `.srv` definition;
2. the matching generated PDU Request/Response packet types from
   `hakoniwa-pdu`;
3. packet base sizes from canonical Hakoniwa offset files;
4. client names, channels, endpoint references, capacities, and heap values.

Output:

```text
build/generated/service/<binding-id>/
├── rpc-server-services.json
└── rpc-client-services.json
```

Both files are projections of one resolved model. The RPC server config contains
server endpoint declarations and the generated static client registrations.
The RPC client config contains the registrations used by the ROS Service Server Node.
This guarantees that static server and client channel assignments agree.

Generation is deterministic, idempotent, and atomic. `--output-dir` overrides
the default CWD-relative location. When `--offset-dir` is omitted,
`HAKO_BINARY_PATH` is used; no implicit system-directory fallback is allowed.

Business Pack places the generated files under:

```text
work/recipes/<recipe-id>/config/service/
```

They are Recipe-specific runtime configuration, not Foundation install
artifacts.

## Client and Channel Allocation

`max_clients` is scoped per Hakoniwa service. The Bridge creates that many RPC
clients, and each client supports one in-flight request.

Client names use:

```text
hakoniwa_pdu_ros_<service-key>_<index>
```

`service-key` is derived from the final component of `hakoniwa_service` and
normalized to lower snake case. For `Service/Add`, the generated clients start
with `hakoniwa_pdu_ros_add_0`.

Channel IDs are logical IDs scoped by service. They restart at zero for every
service:

```text
requestChannelId  = 2 * client_index
responseChannelId = 2 * client_index + 1
```

Therefore client `0` uses channels `0` and `1`; client `1` uses `2` and `3`.
Different service names may reuse the same IDs.

## Runtime Concurrency

The Service Bridge owns one client pool per binding. It assigns an available
client immediately and never queues while all clients are busy. A client is
returned to the pool only after its RPC lifecycle reaches a terminal state.

ROS service callbacks must use the PDU-RPC asynchronous API. RPC worker-thread
completion is transferred to the ROS executor context before completing the
ROS response.

When capacity is exhausted, the Bridge logs a structured `BUSY` rejection and
does not synthesize a successful or zero-filled ROS response. ROS 2 has no
generic service-error response; the ROS client application remains responsible
for its own wait timeout and retry policy.

## Timeout and Shutdown

The Bridge uses the normal PDU-RPC timeout/cancel state machine. It does not
duplicate request IDs or cancellation state.

On timeout:

1. issue cancellation through the PDU-RPC API;
2. wait for the internal terminal result unless transport is already gone;
3. discard a late result after the bridge timeout;
4. release the client only after terminal cleanup;
5. return no fabricated ROS response.

Shutdown stops accepting requests, resolves or cancels active clients through
the normal RPC lifecycle, closes all clients, and then destroys ROS entities.

## Type and Heap Contract

The Bridge maps ROS Request/Response bodies recursively by field name. Generated
converters remain responsible for binary layout and packet headers.

The optional Binding heap uses semantic directions:

```json
{
  "heap": {
    "request_bytes": 4096,
    "response_bytes": 8192
  }
}
```

Both values default to zero and must be non-negative integers. Capacity
overflow must fail explicitly and must not truncate data.

The generator maps request heap to native `pduSize.server.heapSize` and response
heap to `pduSize.client.heapSize`. Runtime validation for different non-zero
values remains dependent on the canonical PDU-RPC/PDU-Python heap contract
tracked in `hakoniwa-pdu-rpc#39`.

## Verification

ROS-independent tests cover strict Binding validation, offset size resolution,
deterministic client/channel generation, heap mapping, role-specific golden
files, atomic writes, and idempotence.

The Docker suite uses Ubuntu 24.04 and ROS 2 Jazzy to resolve the real
`example_interfaces/srv/AddTwoInts` interface and PyPI `hakoniwa-pdu` generated
types, then exercises both the Python API and CLI generation paths.

It also builds a Core-free Endpoint and a PDU-RPC revision with Typed
`call_async()` support. A ROS-independent AddTwoInts RPC fixture uses the
generated RPC server/client configs over real TCP and verifies the complete
`19 + 23 = 42` request/response round trip. The later Service Node E2E reuses
this RPC server and replaces the current Typed RPC client call with the ROS
Service Server Node.

The normal Service Server Node E2E is implemented on this fixture. A real ROS 2
Client calls `/add_two_ints` and receives `42` through the Node, Typed RPC, and
Hakoniwa RPC Server. Concurrency, BUSY, timeout, and shutdown race coverage
remain follow-up work.
