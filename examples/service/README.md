# ROS 2 Service Bridge manual demo

[日本語](README.ja.md)

This walkthrough runs the AddTwoInts Hakoniwa RPC Server, ROS Service Bridge,
and `ros2 service call` as three independently observable processes. It uses the
same Ubuntu 24.04 / ROS 2 Jazzy image as the native integration suite.

## 1. Build and keep a demo container running

Run these commands from the repository root:

```bash
docker build \
  --file test/docker/Dockerfile \
  --tag hakoniwa-pdu-ros-native-test \
  .

docker run --rm --init --detach \
  --name hakoniwa-pdu-ros-service-demo \
  --entrypoint sleep \
  hakoniwa-pdu-ros-native-test infinity
```

Generate the matching RPC Server and Client configs once:

```bash
docker exec hakoniwa-pdu-ros-service-demo bash -lc '
  source /opt/ros/jazzy/setup.bash
  python -m hakoniwa_pdu_ros.generate_service_config \
    --config /workspace/config/service/add_two_ints.json \
    --offset-dir /workspace/test/fixtures/offset \
    --output-dir /tmp/add-two-ints
'
```

## 2. Terminal 1: start the Hakoniwa RPC Server

```bash
docker exec -it hakoniwa-pdu-ros-service-demo bash -lc '
  source /opt/ros/jazzy/setup.bash
  python /workspace/examples/service/add_two_ints_rpc_server.py \
    --service-config /tmp/add-two-ints/rpc-server-services.json \
    --endpoint-config /tmp/add-two-ints/endpoints/server_node.json
'
```

Wait for `AddTwoInts RPC Server started`. Keep this terminal open.

## 3. Terminal 2: start the ROS Service Bridge

```bash
docker exec -it hakoniwa-pdu-ros-service-demo bash -lc '
  source /opt/ros/jazzy/setup.bash
  service-server \
    --config /workspace/config/service/add_two_ints.json \
    --offset-dir /workspace/test/fixtures/offset \
    --output-dir /tmp/add-two-ints
'
```

Wait for the `service ready` diagnostic for `/add_two_ints`. Keep this terminal
open.

## 4. Terminal 3: call the ROS 2 service

```bash
docker exec -it hakoniwa-pdu-ros-service-demo bash -lc '
  source /opt/ros/jazzy/setup.bash
  ros2 service list
  ros2 service type /add_two_ints
  ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 20, b: 22}"
'
```

Expected ROS response:

```text
sum: 42
```

Terminal 1 also prints `20 + 22 = 42`, proving that the request crossed the
ROS Service Bridge and was processed by the Hakoniwa RPC Server.

## 5. Stop and verify cleanup

Press Ctrl+C in Terminal 2, then press Ctrl+C in Terminal 1. Finally remove the
demo container:

```bash
docker stop hakoniwa-pdu-ros-service-demo
```

Verify that it is gone:

```bash
docker ps --filter name=hakoniwa-pdu-ros-service-demo
```

The command must show no running demo container. Do not use broad process-kill
commands; stop the two foreground programs through Ctrl+C first.
