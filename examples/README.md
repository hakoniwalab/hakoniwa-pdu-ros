# Examples

These examples demonstrate the intended minimal workflow:

- Zenoh side sends `Drone/pos`
- `hakoniwa-pdu-ros` bridge publishes `/hakoniwa/drone/pos`
- ROS side publishes `/hakoniwa/drone/cmd`
- `hakoniwa-pdu-ros` bridge forwards it to Zenoh as `Drone/cmd`

## Prerequisites

- ROS 2 environment sourced
- `hakoniwa-pdu-python` available
- `hakoniwa-pdu-endpoint` Python bindings available
- peer-to-peer Zenoh communication on `tcp/127.0.0.1:7447`

Environment example:

```bash
export HAKONIWA_PDU_PYTHON_PATH=/path/to/hakoniwa-pdu-python/src
export HAKONIWA_PDU_ENDPOINT_PYTHON_PATH=/path/to/hakoniwa-pdu-endpoint/build/python
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
source /opt/ros/${ROS_DISTRO}/setup.bash
```

If `HAKONIWA_PDU_ENDPOINT_PYTHON_PATH` points to `build/python`,
`hakoniwa-pdu-ros` also adds the sibling `python/` directory automatically.

## 1. Start the bridge

```bash
ros2 run hakoniwa_pdu_ros bridge --config install/hakoniwa_pdu_ros/share/hakoniwa_pdu_ros/config/sample/sample_binding.json
```

During local development before install:

```bash
python3 -m hakoniwa_pdu_ros --config config/sample/sample_binding.json
```

The bridge listens as a Zenoh peer on `tcp/127.0.0.1:7447`.

## 2. Start the Zenoh peer example

```bash
python3 examples/zenoh_peer.py
```

This example connects to the bridge peer, sends `Drone/pos` periodically, and
prints received `Drone/cmd`.

It uses `config/sample/endpoint_zenoh_connect.json`, while the bridge uses
`config/sample/endpoint_zenoh.json`. The bridge side listens and the example
side connects. In the example-side config, `pos` is send-only and `cmd` is
receive-only, so the sample does not emit unnecessary "No subscribers found"
warnings for `pos`.

## 3. Observe ROS receive path

```bash
python3 examples/ros_pos_subscriber.py
```

You should see `/hakoniwa/drone/pos` updates printed on the ROS side.

## 4. Observe ROS -> Zenoh path

```bash
python3 examples/ros_cmd_publisher.py
```

This publishes `geometry_msgs/msg/Twist` to `/hakoniwa/drone/cmd`.
The Zenoh peer example prints the received `Drone/cmd` payload.
