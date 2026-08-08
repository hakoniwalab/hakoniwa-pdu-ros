# ROS 2 Service Bridge 手動デモ

[English](README.md)

この手順では、AddTwoInts Hakoniwa RPC Server、ROS Service Bridge、
`ros2 service call`を、個別に観測できる3プロセスとして実行します。
環境にはnative integration testと同じUbuntu 24.04 / ROS 2 Jazzy imageを使用します。

## 1. デモ用containerをbuildして起動する

repository rootで実行します。

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

RPC Server/Clientで対応する設定を一度生成します。

```bash
docker exec hakoniwa-pdu-ros-service-demo bash -lc '
  source /opt/ros/jazzy/setup.bash
  python -m hakoniwa_pdu_ros.generate_service_config \
    --config /workspace/config/service/add_two_ints.json \
    --offset-dir /workspace/test/fixtures/offset \
    --output-dir /tmp/add-two-ints
'
```

## 2. ターミナル1: Hakoniwa RPC Serverを起動する

```bash
docker exec -it hakoniwa-pdu-ros-service-demo bash -lc '
  source /opt/ros/jazzy/setup.bash
  python /workspace/examples/service/add_two_ints_rpc_server.py \
    --service-config /tmp/add-two-ints/rpc-server-services.json \
    --endpoint-config /tmp/add-two-ints/endpoints/server_node.json
'
```

`AddTwoInts RPC Server started`を確認し、このターミナルを開いたままにします。

## 3. ターミナル2: ROS Service Bridgeを起動する

```bash
docker exec -it hakoniwa-pdu-ros-service-demo bash -lc '
  source /opt/ros/jazzy/setup.bash
  service-server \
    --config /workspace/config/service/add_two_ints.json \
    --offset-dir /workspace/test/fixtures/offset \
    --output-dir /tmp/add-two-ints
'
```

`/add_two_ints`に対する`service ready`診断を確認し、このターミナルを開いたままにします。

## 4. ターミナル3: ROS 2 Serviceを呼び出す

```bash
docker exec -it hakoniwa-pdu-ros-service-demo bash -lc '
  source /opt/ros/jazzy/setup.bash
  ros2 service list
  ros2 service type /add_two_ints
  ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 20, b: 22}"
'
```

期待するROS response:

```text
sum: 42
```

ターミナル1にも`20 + 22 = 42`と表示されます。これにより、要求がROS Service Bridgeを通り、
Hakoniwa RPC Serverで処理されたことを確認できます。

## 5. 停止とcleanup確認

ターミナル2でCtrl+Cを押し、次にターミナル1でCtrl+Cを押します。最後にcontainerを停止します。

```bash
docker stop hakoniwa-pdu-ros-service-demo
```

containerが残っていないことを確認します。

```bash
docker ps --filter name=hakoniwa-pdu-ros-service-demo
```

実行中のデモcontainerが表示されなければ完了です。広範囲のprocess killは使わず、二つの
foreground programを先にCtrl+Cで通常終了してください。
