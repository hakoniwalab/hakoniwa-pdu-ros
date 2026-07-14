# Zenoh Endpoint Demo Bridge

This example maps the two Zenoh keys used by
`hakoniwa-pdu-endpoint/python/examples/zenoh` to ROS 2 topics.

The endpoint demo publishes raw Hakoniwa PDU payloads on these Zenoh keys:

| Zenoh key | PDU | ROS type | ROS topic |
| --- | --- | --- | --- |
| `hakoniwa/demo/0` | `demo/command` | `std_msgs/msg/UInt16` | `/hakoniwa/demo/command` |
| `hakoniwa/demo/1` | `demo/debuginfo` | `std_msgs/msg/UInt16` | `/hakoniwa/demo/debuginfo` |

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
  --config "${PKG_SHARE}/examples/zenoh/config/zenoh_binding.json"
```

During local development before install:

```bash
python3 -m hakoniwa_pdu_ros \
  --config examples/zenoh/config/zenoh_binding.json
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
/hakoniwa/demo/command [std_msgs/msg/UInt16]
/hakoniwa/demo/debuginfo [std_msgs/msg/UInt16]
```

Then echo the bridged topics:

```bash
ros2 topic echo /hakoniwa/demo/command
```

In another shell, you can also observe:

```bash
ros2 topic echo /hakoniwa/demo/debuginfo
```

Terminal 4: run the endpoint demo responder.

```bash
cd ../hakoniwa-pdu-endpoint
python3 python/examples/zenoh/command_sub.py
```

Terminal 5: run the endpoint demo publisher.

```bash
cd ../hakoniwa-pdu-endpoint
python3 python/examples/zenoh/command_pub.py
```

`command_pub.py` sends `command=1, 2, 3...`.
`command_sub.py` receives each command and sends `debuginfo=command+1000`.
The bridge publishes both values as ROS `std_msgs/msg/UInt16` messages.

## ROS Topic Publish Check

The `examples/zenoh` bridge maps both PDUs from Zenoh to ROS topics, so
`ros2 topic pub` is not required for the normal receive-path demo. It is still
useful to confirm that the ROS graph and message type are working.

In one terminal:

```bash
ros2 topic echo /hakoniwa/demo/command
```

In another terminal:

```bash
ros2 topic pub --once /hakoniwa/demo/command std_msgs/msg/UInt16 "{data: 123}"
```

You should see:

```text
data: 123
```

This checks normal ROS publish/subscribe visibility. It does not send data to
Zenoh, because the `command` binding in this example is `pdu_to_ros`.

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
  --config "${PKG_SHARE}/examples/zenoh/config/zenoh_binding.json"
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

### 5. Run the Publisher on the Mac

If `command_pub.py` runs on the same Mac PC as the router, keep this file as
`127.0.0.1`:

```text
python/examples/zenoh/config/comm/zenoh/mac_client.json5
```

Then run:

```bash
cd ~/project/hakoniwa-pdu-endpoint
python3 python/examples/zenoh/command_pub.py --initial-delay 3
```

### 6. Check ROS Topics

On the ROS machine:

```bash
ros2 topic echo /hakoniwa/demo/command
```

And in another terminal:

```bash
ros2 topic echo /hakoniwa/demo/debuginfo
```

Expected flow:

```text
Mac command_pub.py -> hakoniwa/demo/0 -> /hakoniwa/demo/command
Raspberry Pi command_sub.py -> hakoniwa/demo/1 -> /hakoniwa/demo/debuginfo
```

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
${PKG_SHARE}/examples/zenoh/config/zenoh_binding.json
```

After editing source files under `examples/zenoh/config/`, rebuild and source
the workspace again:

```bash
cd ~/project/ros2_ws
colcon build --packages-select hakoniwa_pdu_ros
source install/setup.bash
PKG_SHARE="$(ros2 pkg prefix hakoniwa_pdu_ros)/share/hakoniwa_pdu_ros"
```
