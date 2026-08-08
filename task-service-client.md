# ROS 2 Service Client Bridge実装タスク

## 1. 目的

箱庭RPC Clientから外部ROS 2 Service Serverを呼び出す逆方向Bridgeを追加する。

```text
Hakoniwa RPC Client
  -> hakoniwa-pdu-rpc TypedRpcServer / RpcMuxServer
  -> HakoniwaRosServiceClientNode
  -> ROS 2 Service Client
  -> ROS 2 Service Server
```

関連Issue:

- `hakoniwa-pdu-ros` #18
- Service generator正規化: `hakoniwa-pdu-ros` #16
- Typed Server API: `hakoniwa-pdu-rpc` #76
- 完了済みService Server Bridge: `hakoniwa-pdu-ros` #10

## 2. 命名と方向

`server`／`client`は既存Service／Actionと同じくROS側から見た役割を表す。

| entry point | ROS側 | 箱庭PDU-RPC側 | Bridgeが所有するEndpoint |
| --- | --- | --- | --- |
| `service-server` | Service Server | Typed RPC Client | `client_endpoint` |
| `service-client` | Service Client | Typed RPC Server | `server_endpoint` |

Service Bindingや生成物は方向を持たない。起動するentry pointだけが方向を決める。

## 3. ユーザー境界

ユーザーが指定するもの:

- ROS Service名
- ROS Service型 `package/srv/Type`
- Hakoniwa Service名
- 同時Client数
- timeout
- Request／Responseの意味的heap上限
- Hakoniwa Client／Server Endpoint参照
- Transport設定への参照
- 自動解決できない場合だけPDU Service型override

ユーザーへ指定させないもの:

- client nameの個別列挙
- channel ID
- packet型名
- base size／offset
- Header field
- raw PDU buffer

## 4. Binding契約

Actionと同じ方向非依存Bindingとする。`kind` fieldは定義せず、含まれていれば
unknown fieldとして拒否する。

Runtime方向は`service-server`／`service-client` entry pointで決定する。同じBindingを
どちらのNodeでも利用でき、Binding内の値によってRuntime方向を切り替えない。

`client_endpoint`／`server_endpoint`はROSの役割ではなく、常にHakoniwa RPC上の役割を表す。
両者はnode IDだけを持ち、endpoint IDとnative設定pathはgeneratorが決定する。

## 5. Generator境界

一つのBindingから両方を生成する。

```text
rpc-server-services.json
rpc-client-services.json
```

ROS側generatorの責務:

- Service Binding validation
- installed ROS `.srv`解決
- generated PDU Service型解決
- semantic heapの意味変換
- PDU-RPC generatorへ渡すmanifest生成
- Recipe-local／repository-local出力先の管理

PDU-RPC generatorの責務:

- client name
- channel ID
- packet型
- native `pduSize`配置
- Endpoint mapping
- native Server／Client config形式
- queue、PDU定義、TCP／tcp_mux transport設定

`hakoniwa-pdu-ros` #16に従い、ROS側はinstalled `hakoniwa-pdu-rpc`
generator APIへの薄いAdapterとする。ROS側が生成する中間manifestには、ROS/PDU型解決で
得たbase sizeを含めるが、native `pduSize`への配置やchannel採番は行わない。

## 6. PDU-RPC Typed Server境界

`hakoniwa-pdu-rpc` #76で複数Service対応Typed Server APIを追加する。

必要な責務:

- `RpcServer`／`RpcMuxServer`の両方を包める
- Service名からgenerated Request／Response型とconverterを解決する
- raw Request packetをtyped Request bodyへ変換する
- typed Response bodyを書き込むbufferを生成する
- Request tokenとclient routingを保持してResponseを送信する
- Cancel eventとcancel replyをtyped APIへ露出する
- Header／metadata／raw bufferをApplicationへ露出しない

複数Serviceは一つのTyped Serverが解決する。

```text
TypedRpcServer
  service("Service/Add")
  service("Service/Reset")
```

またはpoll結果が解決済みService adapterを保持する。同じ情報を二重指定させないことを優先する。

## 7. エラー契約

ROS 2 ServiceにはActionのGoal rejectやCancel protocolがない。Bridgeは既存PDU-RPC Response Headerを使い、異常を箱庭Clientへ明示する。

| 事象 | status | result_code | body |
| --- | --- | --- | --- |
| 正常ROS Response | `DONE` | `OK` | 変換済みResponse |
| Request変換失敗 | `ERROR` | `INVALID` | default Response |
| ROS Service利用不可 | `ERROR` | `NOT_SUPPORTED` | default Response |
| ROS Future例外 | `ERROR` | `ERROR` | default Response |
| Response変換失敗 | `ERROR` | `ERROR` | default Response |
| Hakoniwa Cancel確定 | `DONE` | `CANCELED` | cancel reply |

現在の`TypedRpcClient`はResponse Headerの`status/result_code`を検査せずbodyだけ返す。このままでは異常を呼び出し元へ通知できないため、#76で次を追加する。

- 非`OK` Responseを詳細なtyped例外として通知する。
- 例外は少なくともService名、status、result codeを保持する。
- 正常Responseの外部契約は変更しない。

### CancelとResultの競合

ROS Service要求はremote実行をprotocol cancelできない。ROS Futureの`cancel()`はBridge側の待機を止めるだけで、ROS Serverの処理停止を保証しない。

v1ではPDU-RPC Serverが観測したterminal順序を採用する。

- Cancelが先に確定した場合、ROS Futureをbest-effortでcancelし、cancel replyを返す。遅延ROS Responseは破棄する。
- ROS Responseが先にterminal commitした場合、正常Responseを返す。後続CancelはPDU-RPCの既存競合契約に従う。
- Bridgeは独自の再送、復旧、第二のRPC状態機械を追加しない。

## 8. Runtime

実装候補:

```text
hakoniwa_pdu_ros/service_client_node.py
```

基本フロー:

```text
Typed RPC REQUEST_IN
  -> binding／Serviceを解決
  -> typed Request bodyをROS Requestへ変換
  -> ROS Client.call_async()
  -> ROS Future callback
  -> ROS Responseをtyped Response bodyへ変換
  -> typed send_reply()
```

Runtimeが保持するContextは相関と配送状態だけに限定する。

```text
(service_name, request_token)
  -> ROS Future
  -> typed server event
  -> terminal flag
```

PDU-RPCのService状態、Header、client routing、channelは保持しない。

## 9. テスト

### unit

- `kind`なし方向非依存Binding
- `kind` field拒否
- Request／Response逆方向mapping
- typed error mapping
- generator既存出力互換

### component

- Request -> ROS async call -> Response
- ROS unavailable
- Request／Response変換エラー
- ROS Future例外
- 複数Request
- 複数Service Binding
- Cancel／Response競合
- active Request中shutdown

### Docker E2E

```text
Hakoniwa Typed RPC Client
  -> real TCP / CFFI
  -> HakoniwaRosServiceClientNode
  -> real rclpy Client
  -> AddTwoInts ROS Service Server
```

確認項目:

- `20 + 22 = 42`
- 2回連続Request
- 複数同時Request
- Service unavailable／error応答
- Cancelとlate ROS Response
- shutdown cleanup
- 既存Topic、Service Server、Action両方向の回帰なし

## 10. 実装順序

- [x] ROS側Issue #18を作成する
- [x] RPC側Issue #76を作成する
- [x] generator正規化Issue #16を関連付ける
- [x] Binding、generator、Typed Server、エラー境界を文書化する
- [x] Service Bindingを方向非依存化し、`kind`を削除する
- [x] PDU-RPC Service generator公開APIを実装する
- [x] PDU-RPC複数Service Typed Server APIを実装する
- [x] TypedRpcClientの非OK Response可視化を実装する
- [x] 既存TypeMapperで逆方向Request／Responseを変換する
- [x] `HakoniwaRosServiceClientNode`を実装する
- [x] component testを追加する
- [x] Docker AddTwoInts E2Eを追加する
- [x] `service-client` console entryを追加する
- [x] READMEへService Client手順を追加する
- [x] 全既存testを再実行する

## 11. 完了条件

- 同じService Bindingと生成物を両Runtime方向で使用できる。
- 箱庭AddTwoInts ClientからROS 2 AddTwoInts Serverを呼び出せる。
- 正常Responseと異常Responseを箱庭Clientが区別できる。
- 複数Client／複数Serviceを誤配送なく扱える。
- ROS側がraw packet／Header／channel／offsetを扱わない。
- Cancel／Response競合とshutdownが文書化した契約に従う。
- Docker native suiteで実TCP E2Eを再現できる。
- 既存Topic、Service Server、Action Bridge契約が維持される。
