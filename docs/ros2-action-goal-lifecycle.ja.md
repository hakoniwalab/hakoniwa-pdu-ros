# ROS 2 Action の Goal ライフサイクル

このドキュメントでは、ROS 2 Action を Goal 単位の実行セッションとして整理します。

前提となるユーザー API とデータ定義については、[`ros2-action-user-model.ja.md`](ros2-action-user-model.ja.md) を参照してください。

## 1. 1つの Goal は1回の実行を表す

ROS 2 Action では、1つの Goal が1回の Action 実行を表します。

```text
Goal
  ├── Goal の受理または拒否
  ├── Feedback 0..N回
  ├── Cancel 要求 0..N回
  └── Result 0..1回
```

Goal が受理されると、その Goal に対応する実行状態が作られます。
Feedback、Cancel、Result は、すべて同じ Goal に紐づきます。

同じ Goal を使って別の Action 実行を開始するわけではありません。
新しい実行は、新しい Goal として扱われます。

## 2. Goal の入口では受理または拒否が返る

Action Server は Goal を受信すると、実行開始前に受理するか拒否するかを判断できます。

```python
from rclpy.action import GoalResponse


def goal_callback(goal_request):
    if goal_request.order <= 0:
        return GoalResponse.REJECT

    return GoalResponse.ACCEPT
```

### 2.1 Goal を受理する場合

```text
Action Client                         Action Server
      |                                      |
      | send_goal_async(Goal)                |
      |------------------------------------->|
      |                                      |
      |                    GoalResponse.ACCEPT
      |<-------------------------------------|
      |                                      |
      |                    Goal の実行を開始  |
      |<-------------- Feedback ------------|
      |<-------------- Result --------------|
```

Goal の受理は処理の成功を意味しません。
受理後の実行結果は、`SUCCEEDED`、`CANCELED`、`ABORTED` などの終端状態になります。

### 2.2 Goal を拒否する場合

```text
Action Client                         Action Server
      |                                      |
      | send_goal_async(Goal)                |
      |------------------------------------->|
      |                                      |
      |                    GoalResponse.REJECT
      |<-------------------------------------|
      |                                      |
      |                 ここで通信は終了      |
```

クライアントは Goal Handle の `accepted` を確認します。

```python
def goal_response_callback(future):
    goal_handle = future.result()

    if not goal_handle.accepted:
        return

    result_future = goal_handle.get_result_async()
```

Goal が拒否された場合は、次の処理は発生しません。

- Goal の実行
- Feedback の送信
- Result の返却
- Cancel 対象となる実行状態の作成

Reject は、Action 実行後の失敗ではありません。
Goal を実行対象として受け付けなかったことを表します。

```text
Reject
  Goal を受け付けない
  Result は存在しない

Abort
  Goal を受理して実行する
  実行途中または実行結果として失敗する
  Result が存在する
```

## 3. Action は Goal 中心のセッションである

Service は、Request と Response の1往復を基本単位とします。

```text
Client A -- Request --> Service Server
Client A <-- Response -- Service Server

Client B -- Request --> Service Server
Client B <-- Response -- Service Server
```

このため、Service の実装では、複数クライアントから届く独立した Request を管理するモデルになりやすくなります。

Action では、通信の中心はクライアント接続ではなく Goal です。

```text
Action Client
  └── Goal G1
        ├── Goal Response
        ├── Feedback 0..N
        ├── Cancel Request / Response
        └── Result
```

別のクライアントから Goal が届いた場合も、それぞれ別の Goal セッションとして扱われます。

```text
Client A ── Goal G1 ──┐
                       ├── Action Server
Client B ── Goal G2 ──┘
```

Action Server が複数 Goal を同時実行できるか、実行中の新規 Goal を拒否するか、待ち行列に入れるかは、Action Server の方針です。

ただし、どの方針であっても、Feedback、Cancel、Result は各 Goal に対応付けられます。

## 4. 同一 Goal に対して許される操作

1つの Goal に対して、概念的には次の通信が発生します。

### Client から Server

- Goal の送信: 1回
- Cancel 要求: 0回以上
- Result の取得要求: API上必要に応じて実行

### Server から Client

- Goal 受理・拒否応答: 1回
- Feedback: 0回以上
- Cancel 受理・拒否応答: Cancel 要求ごと
- Result: Goal を受理した場合に最終的に1回

同じ Goal 識別子を使って、もう一度新規 Goal を作成することは想定しません。

```text
Goal G1 を送信
  -> ACCEPT

同じ Goal G1 を新規 Goal として再送
  -> 重複 Goal として扱う
```

Cancel は新しい Action 実行ではありません。
既存 Goal の状態変更を要求する操作です。

## 5. ユーザー定義データと Goal 管理情報

ユーザーが `.action` に定義するデータは次の3種類です。

```text
Goal
Result
Feedback
```

一方、Goal の受理・拒否、Cancel、終了状態、Goal の識別は ROS 2 Action の共通機構です。

```text
ユーザー定義データ
  Goal payload
  Result payload
  Feedback payload

ROS 2 共通管理情報
  Goal ID
  Goal acceptance
  Goal status
  Cancel
  terminal status
```

この分離は、箱庭側の Action 通信プロトコルを検討するときにも重要です。
Action 固有データと、Goal ライフサイクル管理を分離して考える必要があります。

## 6. 次に確認する内容

次の調査では、ROS 2 がこの Goal ライフサイクルを内部でどのような Service、Topic、メッセージ型へ展開しているかを整理します。

確認対象は次のとおりです。

- Send Goal Service
- Get Result Service
- Cancel Goal Service
- Feedback Topic
- Status Topic
- Goal UUID
- GoalStatus
- Goal の受理、拒否、Cancel、終端状態の内部シーケンス
