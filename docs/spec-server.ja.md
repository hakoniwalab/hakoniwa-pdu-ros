# ROS 2 Service Bridge仕様

[English](spec-server.md)

## 目的

本文書は、ROS 2 Service ServerからHakoniwa PDU-RPC Clientへ接続するBridge契約を定義する。

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

逆方向は将来、独立した`HakoniwaRosServiceClientNode`として実装する。このNodeは
ROS Service Clientとして動作し、Hakoniwa側ではRPC Serverになる。両方向を一つのNodeや
一つのBindingに混在させない。

| Binding `kind` | ROS側の役割 | Hakoniwa側の役割 | 現在の状態 |
| --- | --- | --- | --- |
| `ros_service_server` | Service Server | RPC Client | 本仕様の対象 |
| `ros_service_client` | Service Client | RPC Server | 将来、別Node・別Binding契約として追加 |

なお、generatorが出力する`rpc-server-services.json`と
`rpc-client-services.json`のserver/clientはPDU-RPC側の役割であり、Bindingの`kind`とは
別の概念である。現在の`ros_service_server` Bindingでも、Bridgeと接続相手のstatic設定を
一致させるため両方を生成する。

## Service Bindingの正本

独立Service Nodeは[`schema/service-binding.schema.json`](../schema/service-binding.schema.json)
に従うService Bindingを読む。AddTwoIntsの例は
[`config/service/add_two_ints.json`](../config/service/add_two_ints.json)に置く。

利用者が指定する項目:

- ROS側の役割を示す`kind`（現在は`ros_service_server`）
- ROS service名と`package/srv/Type`
- Hakoniwa service名
- RPC client/server Endpoint参照
- serviceごとの`max_clients`
- Bridge timeout
- 任意のrequest/response heap容量
- 必要な場合だけPDU service型override

RPC client名とchannel IDは利用者に指定させない。

`rpc.endpoint_config`の相対pathはService Bindingファイル基準で解決する。Endpoint参照は
そのregistry内に存在しなければならない。不明field、service名や正規化後service-keyの重複、
不正な容量、型解決失敗は生成・起動前に拒否する。

## 設定生成

role別PDU-RPC service configを次のコマンドで生成する。

```bash
python3 -m hakoniwa_pdu_ros.generate_service_config \
  --config config/service/add_two_ints.json \
  --offset-dir /path/to/share/hakoniwa/offset
```

generatorが解決する内容:

1. installed ROS service classと`.srv`定義
2. `hakoniwa-pdu`に含まれる対応PDU Request/Response Packet型
3. Hakoniwa offsetファイルを正本とするPacket base size
4. client名、channel、Endpoint参照、容量、heap

生成物:

```text
build/generated/service/<binding-id>/
├── rpc-server-services.json
└── rpc-client-services.json
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

BridgeはPDU-RPCの通常timeout/cancel state machineを使い、request IDやcancel stateを複製しない。

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

generatorはrequestをnative `pduSize.server.heapSize`、responseを
`pduSize.client.heapSize`へ写す。異なる非ゼロ値のruntime検証は、
`hakoniwa-pdu-rpc#39`で管理するPDU-RPC/PDU-Pythonの正規heap契約確定後に行う。

## 検証

ROS非依存testでは、厳密なBinding validation、offset size解決、client/channel生成、heap mapping、
role別golden file、atomic write、冪等性を確認する。

DockerではUbuntu 24.04 / ROS 2 Jazzy上で実物の`example_interfaces/srv/AddTwoInts`と
PyPI `hakoniwa-pdu`のgenerated型を解決し、Python APIとCLIの両経路を確認する。

さらに、Core不要のEndpointとTyped `call_async()`対応PDU-RPCをDocker内でビルドし、
生成したRPC Server/Client設定を使うAddTwoInts RPC fixtureを実TCPで検証する。このfixtureは
ROS非依存であり、`19 + 23 = 42`のrequest/response往復を確認する。次段階のService Node E2Eでは、
このRPC Serverをそのまま利用し、現在のTyped RPC Client呼び出し部分をROS Service Server
Nodeへ置き換える。

Service Server Nodeの正常系E2Eは、このfixture上で実装済みである。実際のROS 2 Clientから
`/add_two_ints`を呼び、Node、Typed RPC、箱庭RPC Serverを経由して`42`が返ることを確認する。
並列要求、BUSY、timeout、shutdownの異常系・競合系coverageは後続で追加する。
