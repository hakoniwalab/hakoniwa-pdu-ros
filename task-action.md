# ROS 2 Action Bridge実装タスク

## 1. この文書の目的

`hakoniwa-pdu-ros`へROS 2 Action Bridgeを追加するため、ユーザー境界、設定生成、Runtime責務、実装順序、Docker検証条件を一か所で管理する。

最初の対象は、箱庭側にAction Serverが存在し、ROS 2側へAction Serverとして公開する方向である。次フェーズとして逆方向のAction Client Bridgeを実装する。

```text
ROS 2 Action Client
        |
        v
HakoniwaRosActionServerNode    ROS Action Server
        |
        v
hakoniwa-pdu-rpc ActionClient  Hakoniwa Action Client
        |
        v
Hakoniwa Action Server
```

逆方向は次の独立フェーズとする。

```text
ROS 2 Action Server
        ^
        |
HakoniwaRosActionClientNode    ROS Action Client
        ^
        |
hakoniwa-pdu-rpc ActionServer  Hakoniwa Action Server
        ^
        |
Hakoniwa Action Client
```

`server`／`client`というNode名は、既存Service Bridgeと同じく常にROS側から見た役割を表す。Bindingは方向を持たず、起動するNodeのentry pointが方向を決める。

## 2. 初期スコープ

初期実装に含める。

- 両方向から共用できる方向非依存のAction Binding SchemaとFibonacci例
- Action Binding loaderと厳密なvalidation
- BindingからHakoniwa Action manifest／Runtime設定を生成するgenerator
- `HakoniwaRosActionServerNode`
- Bridge生成Hakoniwa `goal_id`とROS Goal UUIDの双方向対応
- Goal accept／reject
- Feedback
- Resultの`SUCCEEDED`／`CANCELED`／`ABORTED`
- 単一Goal Cancel
- 複数同時Goal
- shutdownとactive Goal cleanup
- Docker上のROS 2／Hakoniwa Action E2E

Server側の初期Runtime実装へ混在させない。

- Topic BridgeまたはService Bridgeとの統合Node
- ROS 2 bulk Cancelを箱庭Protocolへ直接追加すること
- ROS 2固有状態を`hakoniwa-pdu-rpc`の状態機械へ追加すること
- Bridge独自のGoal状態機械
- reconnect、Goal resume、Result retention
- shared-memory Action transport
- アプリケーション固有のGoal受理Policyや実処理

## 3. 責務境界

### 3.1 ユーザーが指定するもの

ユーザーはROSと箱庭の意味的なBindingだけを指定する。

- ROS Action名
- ROS Action型 `package/action/Type`
- Hakoniwa Action名
- 同時に通信可能なGoal数
- Goal Response timeout
- 可変長Goal／Result／Feedbackのheap上限
- Hakoniwa Action Client／Server Endpointの参照
- Transport設定への参照
- 自動解決できない場合だけPDU Action型override

### 3.2 generatorが決定するもの

次はユーザーへ手書きさせない。

- Request／Response／Feedback channel ID
- channel名
- packet型名
- Endpoint ID
- slotごとのchannel割当
- resolved heap値
- generated Action Runtime設定のファイル構成

channelとpacketの生成規則は`hakoniwa-pdu-rpc`を正とし、ROS側へ複製しない。

### 3.3 Runtimeが所有するもの

`hakoniwa-pdu-rpc`が所有する。

- Goal、Cancel、Feedback、ResultのProtocol状態
- slot／packet binding
- sequence番号
- Goal／Cancel ResponseとFeedback／ResultのWire順序
- duplicate Goal、slot不足、packet不正、Transport失敗の判定

`hakoniwa-pdu-ros`が所有する。

- ROS Action entityとcallback
- 箱庭Goal送信前の16 byte UUIDv4 `goal_id`生成
- ROS Goal UUID、Hakoniwa `goal_id`、双方のGoal Handleの対応
- ROS Goal／Result／Feedback bodyとgenerated PDU bodyの変換
- ROS executorとHakoniwa pollの接続
- ROS Cancel要求を単一Goal Cancelへ変換する処理
- ROS status／Result APIへの状態反映
- ROS側の診断ログ

BridgeはHakoniwa Action状態を推測せず、`hakoniwa-pdu-rpc`から受けたイベントだけをROS側へ反映する。

## 4. Action Binding仕様

正本を次へ置く。

```text
schema/action-binding.schema.json
config/action/fibonacci.json
config/action/fibonacci-transport.json
```

Service Bindingと同じくROS／箱庭の意味的対応を記述するが、Action Binding自体は方向を持たない。同じBindingと生成物を次の両Nodeから利用する。

| Runtime entry point | ROS側 | Bridge内の箱庭側 | 使用Endpoint |
| --- | --- | --- | --- |
| `action-server` | Action Server | Hakoniwa Action Client | `client_endpoint` |
| `action-client` | Action Client | Hakoniwa Action Server | `server_endpoint` |

Node実装は`action-server`から開始する。`action-client`を追加してもBinding Schemaと生成物を変更しない。

rootは次の形を基本とする。

```json
{
  "$schema": "../../schema/action-binding.schema.json",
  "version": 1,
  "action": {
    "transport_config": "action-transport.json",
    "delta_time_usec": 1000,
    "time_source_type": "real"
  },
  "bindings": [
    {
      "ros_name": "/fibonacci",
      "ros_type": "action_tutorials_interfaces/action/Fibonacci",
      "hakoniwa_action": "fibonacci",
      "pdu_action_type": "sample_action_msgs/Fibonacci",
      "client_endpoint": {
        "node_id": "fibonacci-client"
      },
      "server_endpoint": {
        "node_id": "fibonacci-server"
      },
      "slot_count": 4,
      "goal_response_timeout_msec": 3000,
      "heap": {
        "goal_bytes": 1048576,
        "result_bytes": 1048576,
        "feedback_bytes": 1048576
      }
    }
  ]
}
```

このJSON構造をBinding v1として固定する。

1. `client_endpoint`／`server_endpoint`はbinding単位で指定する。
2. TCP Transportは`action.transport_config`の別ファイルで指定する。
3. `slot_count`は通信lane数であり、Application実行上限ではない。
4. `goal_response_timeout_msec`はGoal Response待ちだけに適用する。
5. heapは`goal_bytes`／`result_bytes`／`feedback_bytes`で指定し、generatorがPDU-RPCの`requestSize`／`responseSize`／`feedbackSize`へ変換する。
6. `pdu_action_type`は`package/Type`形式の任意overrideとし、basenameはROS Action型と一致させる。
7. 複数bindingは同じTransportファイル内のEndpointを参照できる。

`client_endpoint`／`server_endpoint`は常にHakoniwa Action上の役割を表す。Runtime entry pointに応じて意味を反転させない。

```text
action-server
  Bridgeが所有するEndpoint = client_endpoint
  接続相手                 = server_endpoint

action-client
  Bridgeが所有するEndpoint = server_endpoint
  接続相手                 = client_endpoint
```

両Endpointを明示することで、generatorは`hakoniwa-pdu-rpc` Action manifestへ条件分岐なしで写像できる。各Nodeは自身のentry pointに対応するEndpointだけを選ぶ。

## 5. 型解決と設定生成

generatorは次を解決する。

1. installed ROS Action class
2. installed `.action`定義
3. 対応するgenerated PDU Goal／Result／Feedback型とconverter
4. `hakoniwa-pdu-rpc`が要求するAction型 `package/ActionName`
5. Bindingの意味的heap値
6. EndpointとTCP Transport

生成経路は次を原則とする。

```text
Action Binding
    |
    | validate / resolve ROS and PDU types
    v
Hakoniwa Action manifest
    |
    | reuse hakoniwa-pdu-rpc generator contract
    v
resolved-action.json
endpoints.json
queue.json
endpoint files
```

ROS generatorがchannel採番やpacket命名を独自実装してはならない。可能であれば`hakoniwa-pdu-rpc`のgenerator処理をimport可能なAPIとして再利用する。CLI subprocessを使う場合も、入力manifestと生成物の契約をテストで固定する。

生成物の既定出力先はServiceと同じ考え方でrepository-localとする。

```text
build/generated/action/<binding-id>/
```

Business Packから使用する場合はRecipe側が`--output-dir`を指定する。

```text
work/recipes/<recipe-id>/config/action/
```

生成は決定的、冪等、atomicでなければならない。暗黙のsystem directoryへ出力しない。

## 6. Loader

実装候補:

```text
hakoniwa_pdu_ros/action_binding.py
```

loaderは少なくとも次を起動前に拒否する。

- unknown field
- 未対応version
- Runtime方向を表す`kind`など、Binding責務外のfield
- 空のbindings
- 不正または相対形式のROS Action名
- `package/action/Type`でないROS型
- ROS Action名の重複
- Hakoniwa Action名の重複
- Endpoint参照の不整合
- 1未満または整数でない`slot_count`
- 0以下または整数でないtimeout
- 負数または整数でないheap容量
- ROS Action class／`.action`定義の解決失敗
- generated PDU型／converterの欠落または曖昧性

Bindingファイル内の相対pathはBindingファイルの親ディレクトリを基準に解決する。

## 7. Generator

実装候補:

```text
hakoniwa_pdu_ros/action_config_generator.py
hakoniwa_pdu_ros/generate_action_config.py
```

CLI契約案:

```bash
python3 -m hakoniwa_pdu_ros.generate_action_config \
  --config config/action/fibonacci.json \
  --output-dir build/generated/action/fibonacci
```

generatorのContract Testで次を固定する。

- default／alternate Binding
- Runtime方向に依存しない共通Binding
- ROS Action型とPDU Action型の自動解決
- explicit type override
- BindingからHakoniwa Action manifestへの意味変換
- `slot_count`から`slotCount`への変換
- semantic heapから`bufferHeap`への方向別変換
- client／server Endpointの対応
- unknown／missing／type validation
- duplicate名の拒否
- relative pathのBinding基準解決
- deterministic／idempotent／atomic output
- 生成物を`hakoniwa-pdu-rpc` Runtimeが読み込めること

## 8. Runtime実装

実装候補:

```text
hakoniwa_pdu_ros/action_server_node.py
```

NodeはbindingごとにROS Action ServerとHakoniwa Action Clientを生成する。

基本フロー:

```text
ROS Goal Request
  -> BridgeがHakoniwa goal_idをUUIDv4で生成
  -> ROS Goal bodyをPDU Goal bodyへ変換
  -> Hakoniwa send_goal
  -> Goal Response ACCEPTED / REJECTEDをROSへ反映
  -> ACCEPTED後に得られるROS Goal UUIDとHakoniwa goal_idを対応付け
  -> FeedbackをROS GoalHandleへpublish
  -> terminal ResultをROS Result + statusへ変換
```

### 8.1 Goal ID境界

ROS Goal UUIDとHakoniwa `goal_id`は別の識別子とする。BridgeはROSの
`goal_callback`でHakoniwa送信用UUIDv4を生成し、Hakoniwa Goal Responseを
待ってROSのACCEPT／REJECTへ反映する。ROSがGoalを受理した後、
`handle_accepted_callback`でROS Goal UUIDを取得して対応表を確定する。

```text
ROS Goal UUID
    <-> GoalContext
Hakoniwa goal_id
```

`GoalContext`は識別子とHandleの相関だけを所有し、Action状態遷移の正には
ならない。状態の正は`hakoniwa-pdu-rpc`とROS GoalHandleに残す。

- Hakoniwa REJECT／Goal Response timeoutではROS mappingを作らない。
- Hakoniwa ACCEPT後はROS Goal UUIDとHakoniwa Goal Handleを双方向検索可能にする。
- Cancel rejectではContextを維持する。
- terminal ResultをROSへ反映した後、両indexからContextを破棄する。
- shutdownではactive Contextを列挙してcleanupする。
- 同一Hakoniwa `goal_id`または同一ROS Goal UUIDの再登録を拒否する。

Cancel:

```text
ROS Cancel Request
  -> 対応するHakoniwa Goal Handleを検索
  -> send_cancel
  -> Cancel ResponseをROSへ反映
  -> CANCELED Resultを受信してROS Goalをterminal化
```

Cancel Response待ちには`goal_response_timeout_msec`を流用しない。ROS側だけ
timeoutとして`REJECT`した後にHakoniwa側でCancelが受理されると、両Runtimeの
状態が分岐するためである。v1では、対応するactive GoalへのCancel要求は次の
いずれかまで待つ。

- `CANCEL_RESPONSE(ACCEPTED / REJECTED)`
- Cancelより先に確定した`RESULT(SUCCEEDED / ABORTED)`
- Bridge shutdown

Resultが先着した場合はROS Cancelを`REJECT`し、Resultを通常どおりROSへ反映する。
箱庭の`CANCEL_RESPONSE(ACCEPTED)`直後に`RESULT(CANCELED)`が届いた場合は、ROSの
cancel callbackが`ACCEPT`を返してGoalHandleが`CANCELING`になるまでResult配送を
保留する。この保留は両Runtime間の配送順序調整であり、Bridge独自の状態機械には
しない。
shutdown時は待機を解除して`REJECT`する。Transport切断後の状態照会、Cancelの
再送、Bridge独自timeoutはv1へ追加せず、必要になった時点でProtocol回復契約と
合わせて設計する。

実装原則:

- ROS callbackでHakoniwa Result完了まで同期blockしない
- Hakoniwa pollをROS executorへ安全に受け渡す
- Goal ID、状態、Cancel競合をBridge側で二重管理しない
- body変換には既存`TypeMapper`を再利用する
- Header、packet metadata、slotをROS Applicationへ公開しない
- shutdownでは新規Goalを停止し、active GoalとNative handleを安全にcloseする
- NOP、duplicate、late eventを成功イベントとしてROSへ再配送しない
- 同期失敗とProtocol eventを構造化ログで区別する

### 8.2 Action Client Bridge

`HakoniwaRosActionClientNode`は箱庭側へAction Serverを公開し、ROS 2側では
Action Clientとして動作する。

```text
Hakoniwa Action Client
  -> hakoniwa-pdu-rpc Typed Action Server
  -> HakoniwaRosActionClientNode
  -> ROS 2 Action Client
  -> ROS 2 Action Server
```

基本フロー:

```text
Hakoniwa GOAL_REQUEST
  -> typed Goal bodyをROS Goalへ変換
  -> ROS send_goal_async
  -> ROS ACCEPT／REJECTをHakoniwa Goal Responseへ反映
  -> ROS Feedbackをtyped Hakoniwa Feedbackへ変換して送信
  -> ROS terminal statusとResultをHakoniwa Resultへ変換して送信
```

Cancel:

```text
Hakoniwa CANCEL_REQUEST
  -> 対応するROS ClientGoalHandleを検索
  -> ROS cancel_goal_async
  -> ROS Cancel結果をHakoniwa Cancel Responseへ反映
  -> ROS terminal ResultをHakoniwa Resultへ反映
```

方向を追加してもBinding Schema、生成済みAction manifest、Transport設定は変更しない。
Client Nodeは`server_endpoint`を使用する。Goal IDは箱庭Action Clientが生成した
`goal_id`を相関の正として保持し、ROS側が返すClientGoalHandleと対応付ける。

body変換とpacket境界は次の責務に分ける。

- `hakoniwa-pdu-rpc`: Typed Action Server、Header、packet encode/decode、buffer
- `hakoniwa-pdu-ros`: typed Goal／Feedback／Result bodyとROS messageの変換

ROS Bridgeがraw Action packetを直接decode／encodeする実装は採用しない。
`hakoniwa-pdu-rpc` #74で複数Action対応`TypedActionServer`がmainへ統合された。
Client BridgeはこのAPIを利用し、raw packet、Header、slot管理をROS側へ複製しない。

## 9. Dockerテスト環境

macOS host上のROS依存を避け、既存native Docker test基盤を拡張する。

```text
Docker container
  ROS 2 Action Client
  HakoniwaRosActionServerNode
  hakoniwa-pdu-rpc Python CFFI Action Client
  Fibonacci Hakoniwa Action Server fixture
```

最低限のE2E:

1. Goal acceptとFibonacci Result
2. Goal reject
3. Feedback 0回以上とsequence順序
4. Cancel accept後のCANCELED Result
5. Cancel reject後の通常完了
6. ResultとCancelの競合
7. 2回連続Goal
8. `slot_count`までの並行Goal
9. slot上限超過
10. duplicate Goal ID
11. body変換エラー
12. Goal Response timeout
13. 実行中shutdown
14. 複数Action Binding

テスト階層:

```text
pure unit
  loader
  generator
  ROS/PDU body mapper
  ROSとHakoniwa status mapping

component test
  fake ROS GoalHandle + fake ActionClient
  Node lifecycle

Docker E2E
  real ROS 2 Action Client
  real CFFI Action Runtime
  real TCP transport
  Fibonacci Action Server
```

Docker E2Eは成功結果だけでなく、Goal accept／reject、Feedback、Cancel Response、terminal statusを個別に観測して判定する。

## 10. 実装順序

### 10.1 Action Server Bridge

- [x] 最新`hakoniwa-pdu-rpc` Action契約とROS側文書の差分を整理する
- [x] 方向非依存Action Binding fieldを確定する
- [x] `schema/action-binding.schema.json`を追加する
- [x] 共通`config/action/fibonacci.json`とTransport例を追加する
- [x] Action Binding loaderを実装・unit testする
- [x] ROS／PDU Action型resolverを実装・unit testする
- [x] Hakoniwa Action manifest generatorを実装・unit testする
- [x] `hakoniwa-pdu-rpc` generatorとの接続Contract Testを追加する（hakoniwa-pdu-rpc#70）
- [x] ROS Goal UUID／status／body mappingを実装・unit testする
- [ ] `HakoniwaRosActionServerNode`を実装する（Goal／Feedback／Result／単一Goal Cancelまで実装済み。shutdownと異常系を継続）
- [x] Fibonacci Hakoniwa Action Server fixtureを用意する
- [x] Docker正常系E2Eを追加する
- [ ] `sample_action_msgs`を含む`hakoniwa-pdu`公開版へ更新し、Docker E2EのRegistry source fallbackを削除する
- [x] Goal reject E2Eを追加する
- [x] 2回連続Goal E2Eを追加する
- [x] Cancel accept／reject E2Eを追加する
- [x] Result先着とCancelの競合E2Eを追加する
- [x] `slot_count`個のactive Goalと上限超過E2Eを追加する
- [x] Goal Response timeout E2Eを追加する
- [ ] duplicate Goal ID、body変換エラーE2Eを追加する
- [ ] 実行中shutdown E2Eを追加する
- [ ] 複数Action Binding E2Eを追加する
- [ ] README、Action設計文書、package data、console scriptを更新する
- [ ] `hako.py`のbuild／test／install契約へ統合する
- [ ] Business Packへ統合可能な生成先・起動契約を確認する

### 10.2 Action Client Bridge

- [x] 逆方向の責務境界とGoal／Cancelフローを定義する
- [x] Bindingと生成物をServer／Client方向で共用することを確認する
- [x] `hakoniwa-pdu-rpc`へ複数Action対応`TypedActionServer`を追加する（hakoniwa-pdu-rpc#74）
- [x] typed Goal／Feedback／Resultの逆方向mapperを追加する
- [x] `HakoniwaRosActionClientNode`のGoal accept／rejectを実装する
- [x] Feedbackと`SUCCEEDED`／`ABORTED` Resultを実装する
- [x] 単一Goal Cancelと`CANCELED` Resultを実装する
- [x] component testを追加する
- [x] ROS Fibonacci Action Server fixtureを追加する
- [x] Docker正常系E2Eを追加する
- [x] `ABORTED` Resultの実TCP E2Eを追加する
- [x] 複数Goalと複数Action Bindingの相関をcomponent testで固定する
- [x] active GoalとEndpointのshutdown cleanupをcomponent testで固定する
- [x] `action-client` console scriptと利用手順を追加する
- [ ] `hako.py test`のreviewed Action Bridge testへ統合する

## 11. 完了条件

- Bindingだけをユーザー入力として、Hakoniwa Action Runtime設定を再現可能に生成できる。
- 一つのSchema、Binding、生成物を`action-server`／`action-client`の両Runtimeから共用できる。
- ユーザーがchannel ID、packet型、Endpoint IDを手書きしない。
- ROS Action Clientから箱庭Fibonacci Action Serverを呼び、Goal Response、Feedback、Resultを取得できる。
- Cancel accept／rejectとResult競合をROS／Hakoniwa双方の契約どおり扱える。
- 複数Goalとslot上限を決定的に扱える。
- Bridgeが`hakoniwa-pdu-rpc`の状態機械を複製しない。
- Docker上でunit、component、E2E testが再現できる。
- 生成物をrepository-localまたはBusiness Pack Recipe-localへ配置できる。
- `hako.py test`でreviewed Action Bridge testを実行できる。
- READMEからBinding、生成、起動、検証手順へ到達できる。
