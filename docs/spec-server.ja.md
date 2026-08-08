# ROS 2 Service Bridge仕様

[English](spec-server.md)

## 目的

本文書は、ROS 2 ServiceとHakoniwa PDU-RPCを接続する方向非依存のBinding契約と、
ROS Service Server側BridgeのRuntime契約を定義する。

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

Topic Bridgeは既存の独立Nodeと設定を維持する。逆方向RPC、ROS Action、
アプリケーション固有のエラー応答は初期実装の対象外とする。

## Nodeの役割と命名

`server` / `client`は常にROS側から見た役割を表す。現在の実装対象は
`HakoniwaRosServiceServerNode`であり、ROS Service Serverとして要求を受け、
Hakoniwa側ではRPC Clientとして動作する。

逆方向は独立した`HakoniwaRosServiceClientNode`として実装する。このNodeは
ROS Service Clientとして動作し、Hakoniwa側ではRPC Serverになる。両方向を一つのNodeや
一つのRuntimeに混在させない。ただし、Bindingは方向非依存で両Nodeが共有する。

| Runtime entry point | ROS側の役割 | Hakoniwa側の役割 |
| --- | --- | --- |
| `service-server` | Service Server | RPC Client |
| `service-client` | Service Client | RPC Server |

なお、generatorが出力する`rpc-server-services.json`と
`rpc-client-services.json`のserver/clientはPDU-RPC側の役割であり、ROS側のRuntime名とは
別の概念である。一つのBindingから、Bridgeと接続相手のstatic設定を
一致させるため両方を生成する。

## Service Bindingの正本

独立Service Nodeは[`schema/service-binding.schema.json`](../schema/service-binding.schema.json)
に従うService Bindingを読む。AddTwoIntsの例は
[`config/service/add_two_ints.json`](../config/service/add_two_ints.json)に置く。

利用者が指定する項目:

- ROS service名と`package/srv/Type`
- Hakoniwa service名
- RPC client/server Endpointのnode ID
- Transport設定ファイル
- serviceごとの`max_clients`
- Bridge timeout
- 任意のrequest/response heap容量
- 必要な場合だけPDU service型override

RPC client名とchannel IDは利用者に指定させない。

`service.transport_config`の相対pathはService Bindingファイル基準で解決する。Endpoint参照は
そのTransport定義内に存在しなければならない。不明field、service名や正規化後service-keyの重複、
不正な容量、型解決失敗は生成・起動前に拒否する。

## 設定生成

role別PDU-RPC service configを次のコマンドで生成する。

```bash
python3 -m hakoniwa_pdu_ros.generate_service_config \
  --config config/service/add_two_ints.json \
  --offset-dir /path/to/share/hakoniwa/offset
```

ROS側generatorが解決する内容:

1. installed ROS service classと`.srv`定義
2. `hakoniwa-pdu`に含まれる対応PDU Request/Response Packet型
3. Hakoniwa offsetファイルを正本とするPacket base size
4. PDU-RPC generatorへ渡す抽象Service manifest

PDU-RPC側generatorがclient名、channel ID、native `pduSize`、Endpoint ID、queue、
PDU定義、TCP／tcp_mux設定を生成する。ROS側はこれらのnative形式を所有しない。

生成物:

```text
build/generated/service/<binding-id>/
├── hakoniwa-service.json
├── resolved-service.json
├── rpc-server-services.json
├── rpc-client-services.json
├── endpoints.json
├── endpoints/
└── transport/
```

両ファイルは一つのresolved modelから生成する。RPC Server用にはserver Endpointとstatic client
登録を含め、RPC Client用にはROS Service Server Nodeが利用するclient登録を含める。これにより、
static server/client間のclient名とchannel割当を一致させる。

生成は決定的・冪等・atomicである。`--output-dir`でCWD基準の既定出力先を変更できる。
`--offset-dir`省略時は`HAKO_BINARY_PATH`を使い、暗黙のsystem directoryへfallbackしない。

Business Packでは次へ配置する。

```text
work/recipes/<recipe-id>/config/service/
```

生成物はRecipe固有runtime configであり、Foundation install artifactではない。

## Clientとchannelの採番

`max_clients`はHakoniwa serviceごとの値である。Bridgeはその数のRPC clientをpoolとして持ち、
各clientは同時に一要求を処理する。

client名:

```text
hakoniwa_pdu_ros_<service-key>_<index>
```

`service-key`は`hakoniwa_service`の末尾をlower snake caseへ正規化する。
`Service/Add`では`hakoniwa_pdu_ros_add_0`から始まる。

channel IDはserviceごとの論理IDであり、各serviceで0から再開する。

```text
requestChannelId  = 2 * client_index
responseChannelId = 2 * client_index + 1
```

client `0`は`0`と`1`、client `1`は`2`と`3`を使う。service名が異なれば同じIDを利用できる。

## Runtimeの並列性

Service Bridgeはbindingごとのclient poolを持つ。空きclientへ要求を直ちに割り当て、
すべて使用中の場合はqueueへ入れない。clientはRPC lifecycleがterminal stateになった後だけpoolへ戻す。

ROS service callbackはPDU-RPC async APIを利用する。RPC worker threadの完了はROS executor
contextへ受け渡してからROS responseを完了する。

容量超過時は構造化`BUSY`ログを出し、成功応答やゼロ値応答を合成しない。ROS 2 Serviceには
汎用エラー応答がないため、ROS client applicationが自身のwait timeoutとretry policyを持つ。

## Timeoutとshutdown

Bridgeがdeadlineを一元管理し、PDU-RPC呼び出し自体は`timeout_usec=0`（無期限待ち）で
開始する。期限到達時はPDU-RPCの通常cancel state machineを使い、request IDやcancel stateを
複製しない。BridgeとPDU-RPCへ同じtimeoutを二重設定して、cancelを競合させてはならない。

timeout時:

1. PDU-RPC APIからcancelする
2. transport切断済みでなければ内部terminal resultを待つ
3. Bridge timeout後のlate resultは破棄する
4. terminal cleanup後にclientを解放する
5. ROSの擬似応答を返さない

shutdownでは新規要求を止め、active clientを通常RPC lifecycleで完了またはcancelし、
全clientをcloseしてからROS entityを破棄する。

## 型とheapの契約

BridgeはROS Request/Response bodyをfield名で再帰変換する。binary layoutとPacket headerは
generated converterの責務とする。

Bindingの任意heapは意味方向で記述する。

```json
{
  "heap": {
    "request_bytes": 4096,
    "response_bytes": 8192
  }
}
```

既定値は両方0で、0以上の整数とする。容量超過は明示的に失敗させ、切り詰めない。

PDU-RPC runtimeはrequest容量を`pduSize.client.heapSize`、response容量を
`pduSize.server.heapSize`として解釈する。generatorはこのnative namingを利用者へ露出させず、
`request_bytes`を`client.heapSize`、`response_bytes`を`server.heapSize`へ変換するAdapterとする。
PDU-Pythonの汎用service PDU definition builderとの命名不整合は`hakoniwa-pdu-rpc#39`で管理するが、
本Bridgeが生成してPDU-RPC runtimeへ直接渡す設定の方向はこの契約で固定する。

## 検証

ROS非依存testでは、厳密なBinding validation、offset size解決、client/channel生成、heap mapping、
role別golden file、atomic write、冪等性を確認する。

DockerではUbuntu 24.04 / ROS 2 Jazzy上で実物の`example_interfaces/srv/AddTwoInts`と
PyPI `hakoniwa-pdu`のgenerated型を解決し、Python APIとCLIの両経路を確認する。

さらに、Core不要のEndpointとTyped `call_async()`対応PDU-RPCをDocker内でビルドし、
生成したRPC Server/Client設定を使うAddTwoInts RPC fixtureを実TCPで検証する。このfixtureは
ROS非依存であり、`19 + 23 = 42`のrequest/response往復を確認する。Service Node E2Eも同じ
RPC Serverを利用する。

Service Server NodeのE2Eは、このfixture上で実装済みである。実際のROS 2 Clientから
`/add_two_ints`を呼び、Node、Typed RPC、箱庭RPC Serverを経由する正常応答、2回の連続呼び出し、
4件の並列要求、5件目の`BUSY`拒否、timeout後のlate normal response破棄、同じclientの再利用を
確認する。さらに、要求処理中のshutdownが通常のRPC cancelを送り、terminal cleanupを待って
poolを閉じ、ROS応答を合成しないことを確認する。

client poolの容量上限、`BUSY`、release後の再利用、shutdown時cancel/closeはROS非依存unit testで
確認する。複数の独立した`RpcClient`接続を受けるRPC Server transportには、通常の単一接続TCP
serverではなく`tcp_mux`を使う。pinned PDU-RPCのPython `RpcMuxServer`がaccept lifecycleと、
受理した各接続に対するRPC Server adapterを隠蔽する。Docker E2EではこのAPIにより、
`max_clients=4`までの実TCP並列処理と、容量超過時にROS clientへ応答を合成せず構造化`BUSY`ログを
出すことを確認する。
