# Zenoh Endpoint Demo Bridge

This example maps the two Zenoh keys used by
`hakoniwa-pdu-endpoint/python/examples/zenoh` to ROS 2 topics.

The bidirectional demo uses these Zenoh keys and ROS topics:

| Zenoh key | PDU | ROS type | ROS topic |
| --- | --- | --- | --- |
| `hakoniwa/demo/0` | `demo/command` | `std_msgs/msg/UInt16` | `/pdu/demo/command`, `/demo/command` |
| `hakoniwa/demo/1` | `demo/debuginfo` | `std_msgs/msg/UInt16` | `/pdu/demo/debuginfo`, `/demo/debuginfo` |

When `direction` and `topic` are omitted in the binding config, the bridge
uses `/<robot>/<pdu>` as the ROS-owned topic and creates a PDU-owned mirror
under `/pdu`:

- `pdu_to_ros`: `/pdu/<robot>/<pdu>`
- `ros_to_pdu`: `/<robot>/<pdu>`

For this demo:

| Owner | ROS topic | Bridge action | Typical use |
| --- | --- | --- | --- |
| ROS | `/demo/command` | subscribe and send to Zenoh key `hakoniwa/demo/0` | send command from ROS |
| PDU | `/pdu/demo/command` | publish from Zenoh key `hakoniwa/demo/0` | observe command sent by the PDU side |
| ROS | `/demo/debuginfo` | subscribe and send to Zenoh key `hakoniwa/demo/1` | send debuginfo from ROS, if needed |
| PDU | `/pdu/demo/debuginfo` | publish from Zenoh key `hakoniwa/demo/1` | read debuginfo returned by the responder |

The `/pdu/...` namespace is reserved for PDU-owned mirror topics. The bridge
does not subscribe to those topics, so publishing to `/pdu/...` from ROS is
ignored by the bridge.

```json
{
  "pdu_key": {
    "robot_name": "demo",
    "pdu_name": "command"
  }
}
```

## Prerequisites

- ROS 2 environment sourced
- `hakoniwa-pdu` available
- `hakoniwa-pdu-endpoint` Python bindings available
- `zenohd` available on `PATH`

Example environment:

macOS:

```bash
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
source /opt/ros/${ROS_DISTRO}/setup.bash

export HAKONIWA_PDU_ENDPOINT_PYTHON_PATH=~/project/hakoniwa-pdu-endpoint/build-zenoh-shared/python
export HAKO_PDU_ENDPOINT_LIB_DIR=~/project/hakoniwa-pdu-endpoint/build-zenoh-shared/src
export HAKO_PDU_ENDPOINT_SHARED_LIB=~/project/hakoniwa-pdu-endpoint/build-zenoh-shared/src/libhakoniwa_pdu_endpoint.dylib
```

Linux / Raspberry Pi:

```bash
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
source /opt/ros/${ROS_DISTRO}/setup.bash

export HAKONIWA_PDU_ENDPOINT_PYTHON_PATH=~/project/hakoniwa-pdu-endpoint/build-zenoh-shared/python
export HAKO_PDU_ENDPOINT_LIB_DIR=~/project/hakoniwa-pdu-endpoint/build-zenoh-shared/src
export HAKO_PDU_ENDPOINT_SHARED_LIB=~/project/hakoniwa-pdu-endpoint/build-zenoh-shared/src/libhakoniwa_pdu_endpoint.so
export LD_LIBRARY_PATH=~/project/hakoniwa-pdu-endpoint/build-zenoh-shared/src:${LD_LIBRARY_PATH:-}
```

If you are using a local `hakoniwa-pdu-python` checkout instead of the
installed `hakoniwa-pdu` package, also set:

```bash
export HAKONIWA_PDU_PYTHON_PATH=/path/to/hakoniwa-pdu-python/src
```

## Build and Source

Build `hakoniwa_pdu_ros` with `colcon` so the example config files are copied
under the installed package share directory.

```bash
mkdir -p ~/project/ros2_ws/src
ln -s ~/project/hakoniwa-pdu-ros ~/project/ros2_ws/src/hakoniwa-pdu-ros

cd ~/project/ros2_ws
colcon build --packages-select hakoniwa_pdu_ros
source install/setup.bash

PKG_SHARE="$(ros2 pkg prefix hakoniwa_pdu_ros)/share/hakoniwa_pdu_ros"
```

After editing files under `examples/zenoh/config/`, run `colcon build` again
and re-source `install/setup.bash` before using the installed config files.

## Run

Use five terminals.

Terminal 1: start the Zenoh router.

```bash
zenohd -c "${PKG_SHARE}/examples/zenoh/config/comm/zenoh/router.json5"
```

Terminal 2: start the ROS bridge.

```bash
ros2 run hakoniwa_pdu_ros bridge \
  --config "${PKG_SHARE}/examples/zenoh/config/zenoh_bidirectional_binding.json"
```

During local development before install:

```bash
python3 -m hakoniwa_pdu_ros \
  --config examples/zenoh/config/zenoh_bidirectional_binding.json
```

Terminal 3: observe the ROS topics.

First check that the bridge is visible in the ROS graph:

```bash
ros2 node list
ros2 topic list -t
```

You should see:

```text
/hakoniwa_pdu_ros_bridge
/pdu/demo/command [std_msgs/msg/UInt16]
/demo/command [std_msgs/msg/UInt16]
/pdu/demo/debuginfo [std_msgs/msg/UInt16]
/demo/debuginfo [std_msgs/msg/UInt16]
```

Then echo the response topic:

```bash
ros2 topic echo /pdu/demo/debuginfo
```

In another shell, check that the command topic exists:

```bash
ros2 topic info /demo/command
```

Terminal 4: run the endpoint demo responder.

```bash
cd ../hakoniwa-pdu-endpoint
python3 python/examples/zenoh/command_sub.py
```

Terminal 5: publish a command from ROS.

```bash
ros2 topic pub --once /demo/command std_msgs/msg/UInt16 "{data: 123}"
```

`command_sub.py` receives the command over Zenoh and sends
`debuginfo=command+1000`. The bridge publishes the response to
`/pdu/demo/debuginfo`, so the expected ROS output is:

```text
data: 1123
```

You can also send `demo/debuginfo` to Zenoh through `/demo/debuginfo`,
but it is not needed for this command/response demo.

## Receive-Only Variant

If you only want to observe the two Zenoh keys as ROS topics, explicitly set
`direction` to `pdu_to_ros` and use the receive-only config:

```bash
ros2 run hakoniwa_pdu_ros bridge \
  --config "${PKG_SHARE}/examples/zenoh/config/zenoh_binding.json"
```

That variant maps both `command` and `debuginfo` from Zenoh to ROS topics:

```text
hakoniwa/demo/0 -> /pdu/hakoniwa/demo/command
hakoniwa/demo/1 -> /pdu/hakoniwa/demo/debuginfo
```

## Production Setup

In a real Mac + Raspberry Pi setup, keep the Zenoh router on the Mac PC and
make every client connect to the Mac IP address.

### 1. Get the Mac IP Address

On the Mac PC that runs `zenohd`, check the active LAN IP address.

For Wi-Fi:

```bash
ipconfig getifaddr en0
```

For wired Ethernet, try:

```bash
ipconfig getifaddr en1
```

If you are not sure which interface is active, use:

```bash
route get default | grep interface
ifconfig
```

In the examples below, replace `192.168.0.10` with the Mac IP address you
found.

### 2. Start the Router on the Mac

On the Mac PC:

```bash
zenohd -c "${PKG_SHARE}/examples/zenoh/config/comm/zenoh/router.json5"
```

The router config listens on all interfaces:

```text
examples/zenoh/config/comm/zenoh/router.json5
tcp/0.0.0.0:7447
```

### 3. Point the ROS Bridge to the Mac Router

If the ROS bridge runs on the same Mac PC as the router, no change is needed:

```text
examples/zenoh/config/comm/zenoh/client.json5
tcp/127.0.0.1:7447
```

If the ROS bridge runs on another machine, edit this file:

```text
examples/zenoh/config/comm/zenoh/client.json5
```

Change:

```json5
"tcp/127.0.0.1:7447"
```

to:

```json5
"tcp/192.168.0.10:7447"
```

Rebuild the ROS workspace so the edited config is installed:

```bash
cd ~/project/ros2_ws
colcon build --packages-select hakoniwa_pdu_ros
source install/setup.bash
PKG_SHARE="$(ros2 pkg prefix hakoniwa_pdu_ros)/share/hakoniwa_pdu_ros"
```

Then start the bridge on the ROS machine:

```bash
ros2 run hakoniwa_pdu_ros bridge \
  --config "${PKG_SHARE}/examples/zenoh/config/zenoh_bidirectional_binding.json"
```

### 4. Point the Raspberry Pi Demo to the Mac Router

On the Raspberry Pi side, edit the endpoint demo config in
`hakoniwa-pdu-endpoint`:

```text
python/examples/zenoh/config/comm/zenoh/raspberry_pi_client.json5
```

Change:

```json5
"tcp/127.0.0.1:7447"
```

to:

```json5
"tcp/192.168.0.10:7447"
```

Then run the responder on the Raspberry Pi:

```bash
cd ~/project/hakoniwa-pdu-endpoint
python3 python/examples/zenoh/command_sub.py
```

### 5. Send a Command from ROS

On the ROS machine:

```bash
ros2 topic pub --once /demo/command std_msgs/msg/UInt16 "{data: 123}"
```

The Raspberry Pi responder should receive `command=123` and send
`debuginfo=1123`.

### 6. Check ROS Response Topic

On the ROS machine:

```bash
ros2 topic echo /pdu/demo/debuginfo
```

Expected flow:

```text
ROS /demo/command -> hakoniwa/demo/0 -> Raspberry Pi command_sub.py
Raspberry Pi command_sub.py -> hakoniwa/demo/1 -> ROS /pdu/demo/debuginfo
```

### 7. Optional: Run the Mac Publisher

If you also want to publish `command=1, 2, 3...` from the endpoint demo on the
Mac, keep this file as `127.0.0.1` when it runs on the same Mac PC as the
router:

```text
python/examples/zenoh/config/comm/zenoh/mac_client.json5
```

```bash
cd ~/project/hakoniwa-pdu-endpoint
python3 python/examples/zenoh/command_pub.py --initial-delay 3
```

To observe that `command_pub.py` input on ROS, use the receive-only bridge
variant described above.

## Troubleshooting

### Bridge Started but No Node or Topics Appear

The bridge node name is:

```text
/hakoniwa_pdu_ros_bridge
```

If `ros2 node list` or `ros2 topic list -t` does not show it, restart the ROS
daemon and check again:

```bash
ros2 daemon stop
ros2 daemon start
ros2 node list --include-hidden-nodes
ros2 topic list -t
```

This can matter after switching RMW implementations, for example after using
`rmw_zenoh` and then returning to the default RMW.

Also compare these environment variables between the bridge terminal and the
terminal running `ros2 node list`:

```bash
echo "ROS_DISTRO=${ROS_DISTRO:-}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-}"
echo "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-}"
```

The values must be compatible across terminals. A different `ROS_DOMAIN_ID` is
enough to hide the bridge from `ros2 node list`.

### Config Changes Do Not Take Effect

When using `ros2 run`, the config is read from the installed package share
directory:

```text
${PKG_SHARE}/examples/zenoh/config/zenoh_bidirectional_binding.json
```

After editing source files under `examples/zenoh/config/`, rebuild and source
the workspace again:

```bash
cd ~/project/ros2_ws
colcon build --packages-select hakoniwa_pdu_ros
source install/setup.bash
PKG_SHARE="$(ros2 pkg prefix hakoniwa_pdu_ros)/share/hakoniwa_pdu_ros"
```

### `/pdu` Is Reserved for PDU-Owned Topics

The bridge reserves `/pdu/...` for PDU-owned mirror topics. Do not use `/pdu`
as the configured ROS-owned `topic`.

This is invalid:

```json
{
  "pdu_key": {"robot_name": "demo", "pdu_name": "command"},
  "direction": "ros_to_pdu",
  "topic": "/pdu/demo/command"
}
```

Use the ROS-owned topic instead, or omit `topic`:

```text
/demo/command
```
