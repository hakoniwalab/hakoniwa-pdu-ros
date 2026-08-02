# ROS 2 Action のユーザー視点モデル

このドキュメントでは、ROS 2 Action をアプリケーション開発者から見た形で整理します。

ここでは、箱庭側の通信プロトコルや内部実装には踏み込みません。まず、ROS 2 の利用者が何を定義し、どの API を呼び、どのようなデータを送受信するのかを明確にすることを目的とします。

この整理を出発点として、後続の設計で以下を判断します。

- 箱庭側で必要な通信イベントは何か
- 既存の Service 用プロトコルをどこまで再利用できるか
- Action 専用プロトコルとして何を新設する必要があるか

## 1. ユーザーが定義するもの

ROS 2 Action の型は、`.action` ファイルに次の3つのデータ構造を定義します。

```text
Goal
---
Result
---
Feedback
```

たとえば、単純な Fibonacci Action は次のように定義されます。

```text
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

ユーザーが定義するのは、Action 固有のデータだけです。

| 区分 | 意味 | 通信方向 |
| --- | --- | --- |
| Goal | 処理開始時に渡す入力データ | Client から Server |
| Result | 処理完了時に返す最終結果 | Server から Client |
| Feedback | 処理途中に返す進捗データ | Server から Client |

ユーザーは `.action` ファイル内では、次の情報を定義しません。

- Goal ID
- Goal の受理・拒否
- Goal の状態
- Cancel 要求
- Action の終了状態

これらは ROS 2 Action の共通機構として提供されます。

## 2. ROS 2 がユーザー定義の周囲に追加するもの

ROS 2 では、1回の Action 実行を Goal Handle で扱います。

Goal Handle は、送信した Goal 1件を表し、クライアントはこれを通して次の操作を行います。

- Goal が受理されたか確認する
- Goal に対する Feedback を受信する
- Goal の Cancel を要求する
- Goal の最終 Result を取得する

概念的には、ROS 2 はユーザー定義データに共通管理情報を組み合わせています。

```text
Goal 送信
  Goal の識別情報
  + ユーザー定義 Goal

Feedback 通知
  Goal の識別情報
  + ユーザー定義 Feedback

Result 取得
  共通の終了状態
  + ユーザー定義 Result
```

このドキュメントでは API 利用者から見える範囲を扱います。ROS 2 が内部で生成する Service、Topic、メッセージ型については、別途ワイヤプロトコルの設計資料で整理します。

## 3. Action Client の使い方

Action Client は、利用者から見ると主に次の操作を行います。

1. Action Server の起動を待つ
2. Goal を送信する
3. Goal の受理・拒否を確認する
4. Feedback を受信する
5. Result を取得する
6. 必要に応じて Cancel を要求する

以下では `rclpy` を使った Python の例を示します。

## 3.1 Action Client を生成する

```python
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci

self._client = ActionClient(
    self,
    Fibonacci,
    '/fibonacci',
)
```

ユーザーが指定するのは次の2点です。

- Action 型
- Action 名

## 3.2 Action Server を待つ

```python
self._client.wait_for_server()
```

指定した Action 名と型に対応する Action Server が利用可能になるまで待ちます。

## 3.3 Goal データを作成する

```python
goal = Fibonacci.Goal()
goal.order = 10
```

ユーザーが作成するのは、`.action` ファイルの Goal セクションから生成されたデータです。

Goal ID や通信管理情報は、ユーザーが直接設定しません。

## 3.4 Goal を送信する

```python
send_goal_future = self._client.send_goal_async(
    goal,
    feedback_callback=self.feedback_callback,
)

send_goal_future.add_done_callback(
    self.goal_response_callback,
)
```

Goal 送信は非同期です。

`send_goal_async()` が返す Future は、最終 Result を表すものではありません。これは、Server が Goal を受理したか、拒否したかを受け取るための Future です。

```text
Goal 送信
  -> Goal 受理
  -> Goal 拒否
```

## 3.5 Goal の受理・拒否を確認する

```python
def goal_response_callback(self, future):
    goal_handle = future.result()

    if not goal_handle.accepted:
        self.get_logger().info('Goal was rejected')
        return

    self.get_logger().info('Goal was accepted')

    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(self.result_callback)
```

Goal が受理されたことと、処理が成功したことは別です。

受理された Goal は、その後に次のいずれかの状態になります。

- 成功
- Cancel
- Abort

## 3.6 Feedback を受信する

```python
def feedback_callback(self, feedback_message):
    feedback = feedback_message.feedback

    self.get_logger().info(
        f'Partial sequence: {feedback.partial_sequence}'
    )
```

Feedback には次の特徴があります。

- 0回以上通知される
- 特定の Goal に対応する
- 通知されても Goal は終了しない
- ユーザー定義の Feedback データを含む

コールバックに渡される値はラッパーメッセージであり、Action 固有の Feedback は `feedback` フィールドから取得します。

## 3.7 Result を取得する

```python
def result_callback(self, future):
    wrapped_result = future.result()

    status = wrapped_result.status
    result = wrapped_result.result

    self.get_logger().info(f'Status: {status}')
    self.get_logger().info(f'Result: {result.sequence}')
```

最終応答には、概念的に次の2種類の情報が含まれます。

- ROS 2 Action 共通の終了状態
- ユーザー定義の Result データ

Result は、最初の `send_goal_async()` から直接返るのではありません。

```text
send_goal_async()
  -> Goal Handle を取得
  -> Goal Handle に対して get_result_async()
  -> 最終 Result を取得
```

## 3.8 Cancel を要求する

```python
cancel_future = goal_handle.cancel_goal_async()
```

Cancel は要求です。

`cancel_goal_async()` を呼んだ時点で、Goal が即座に Cancel 済みになるわけではありません。

Server は Cancel 要求を受理または拒否します。また、Cancel 要求を受理した後も、実際の処理を停止して終端状態に到達するまで時間がかかる場合があります。

クライアント視点では、次の2つを分けて扱う必要があります。

```text
Cancel 要求
  -> Cancel 要求の受理または拒否

Goal 実行
  -> 最終的に Canceled などの終端状態へ到達
```

## 4. Action Server の使い方

Action Server は、クライアントから送られた Goal を受け取り、処理を実行します。

ユーザーは主に次の処理を実装します。

- Goal を受理するか拒否するか
- Cancel 要求を受理するか拒否するか
- Goal の実処理
- Feedback の送信
- Result の返却
- 成功、Cancel、Abort の終端状態設定

## 4.1 Action Server を生成する

```python
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

self._server = ActionServer(
    self,
    Fibonacci,
    '/fibonacci',
    execute_callback=self.execute_callback,
)
```

最小構成では、ユーザーは次の3点を指定します。

- Action 型
- Action 名
- Goal 実行コールバック

## 4.2 Goal の受理・拒否を判断する

必要に応じて Goal コールバックを指定できます。

```python
from rclpy.action import GoalResponse

def goal_callback(self, goal_request):
    if goal_request.order <= 0:
        return GoalResponse.REJECT

    return GoalResponse.ACCEPT
```

ここで受け取る `goal_request` は、ユーザーが `.action` ファイルに定義した Goal データです。

Server は実行開始前に Goal を拒否できます。

## 4.3 Goal を実行する

```python
def execute_callback(self, goal_handle):
    goal = goal_handle.request

    sequence = [0, 1]

    for index in range(2, goal.order):
        sequence.append(sequence[-1] + sequence[-2])

    result = Fibonacci.Result()
    result.sequence = sequence

    goal_handle.succeed()
    return result
```

実行コールバックでは、Goal Handle を通じて次の操作を行います。

- Goal データの取得
- Feedback の送信
- Cancel 要求の確認
- 終端状態の設定
- Result の返却

## 4.4 Feedback を送信する

```python
feedback = Fibonacci.Feedback()
feedback.partial_sequence = sequence

goal_handle.publish_feedback(feedback)
```

Server が送信するのは、ユーザー定義の Feedback データです。

どの Goal に対する Feedback かという管理は、Goal Handle と ROS 2 Action の共通機構が担当します。

## 4.5 Cancel 要求を受け付ける

必要に応じて Cancel コールバックを指定します。

```python
from rclpy.action import CancelResponse

def cancel_callback(self, goal_handle):
    return CancelResponse.ACCEPT
```

Cancel 要求を受理した場合、実行処理側でも Cancel 要求を確認し、処理を終了する必要があります。

```python
if goal_handle.is_cancel_requested:
    goal_handle.canceled()

    result = Fibonacci.Result()
    result.sequence = sequence
    return result
```

Cancel 要求を受理する処理と、Goal を実際に Canceled 状態へ遷移させる処理は別です。

## 4.6 Goal の終了状態を設定する

Action Server は Goal の終了時に、次のいずれかを設定します。

### 成功

```python
goal_handle.succeed()
```

### Cancel

```python
goal_handle.canceled()
```

### Abort

```python
goal_handle.abort()
```

その後、ユーザー定義の Result データを返します。

```python
return result
```

終了状態と Result データは別の情報です。

```text
終了状態
  SUCCEEDED
  CANCELED
  ABORTED

Result
  ユーザー定義データ
```

## 5. ユーザー視点の通信シーケンス

正常終了する場合の基本フローは次のとおりです。

```text
Action Client                              Action Server
      |                                          |
      | Goal データを生成                         |
      |                                          |
      | send_goal_async(Goal)                    |
      |----------------------------------------->|
      |                                          |
      |                    Goal を受理または拒否  |
      |<-----------------------------------------|
      |                                          |
      | Goal Handle を取得                        |
      |                                          |
      |                    Goal の処理を実行      |
      |                                          |
      |<------------- Feedback -----------------|
      |<------------- Feedback -----------------|
      |                                          |
      | get_result_async()                       |
      |----------------------------------------->|
      |                                          |
      |<--------- Status + Result ---------------|
      |                                          |
```

Cancel が発生する場合は、次の流れが追加されます。

```text
Action Client                              Action Server
      |                                          |
      | cancel_goal_async()                      |
      |----------------------------------------->|
      |                                          |
      |             Cancel 要求を受理または拒否  |
      |<-----------------------------------------|
      |                                          |
      |             実処理を停止                 |
      |             Goal を CANCELED に変更      |
      |                                          |
      |<------ CANCELED + Result ----------------|
      |                                          |
```

## 6. ユーザーが直接扱うデータと扱わないデータ

### ユーザーが定義・操作するもの

| 項目 | Client | Server |
| --- | --- | --- |
| Goal データ | 作成して送信 | 受信して参照 |
| Feedback データ | 受信 | 作成して送信 |
| Result データ | 受信 | 作成して返却 |
| Goal 受理・拒否 | 結果を確認 | 判断する |
| Cancel | 要求する | 受理・拒否し、処理を停止する |
| 終了状態 | 確認する | 設定する |

### ROS 2 が管理するもの

- Goal の一意な識別
- Goal Handle
- Goal と Feedback の対応付け
- Goal と Result の対応付け
- Goal 状態の配送
- Cancel 要求の通信
- Action の内部 Service / Topic 構成

## 7. Service とのユーザー視点の違い

Service は基本的に1回の Request と1回の Response で完了します。

```text
Request
  -> Response
```

Action は、ユーザー視点でも複数段階に分かれています。

```text
Goal 送信
  -> Goal 受理または拒否
  -> 0回以上の Feedback
  -> 必要に応じて Cancel
  -> 終了状態 + Result
```

そのため、Action は単純な Service の Request / Response 拡張として扱えるとは限りません。

少なくともユーザー視点では、次の概念が独立しています。

- Goal の送信
- Goal の受理・拒否
- Goal の実行状態
- Feedback
- Cancel 要求
- Cancel 要求の受理・拒否
- 最終終了状態
- Result

## 8. 次に整理する内容

このユーザー視点モデルを基に、次のドキュメントでは ROS 2 Action の内部通信構造を整理します。

主な確認対象は次のとおりです。

- `.action` から生成される型
- Send Goal Service
- Get Result Service
- Cancel Goal Service
- Feedback Topic
- Status Topic
- Goal UUID
- GoalStatus と状態遷移
- Client と Server の内部通信シーケンス

その分析結果を踏まえて、箱庭側の Action プロトコルをデータ型から独立して設計します。
