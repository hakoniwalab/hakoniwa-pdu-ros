# 箱庭 Action PDU データモデル案

このドキュメントでは、ROS 2 の `.action` ファイルから箱庭 Action 用 PDU を生成する際のデータモデル案を整理します。

対象は、ユーザーが定義する Goal / Result / Feedback と、それらを箱庭上で通信するための共通ヘッダです。

## 1. 基本方針

`.action` の3区画は、そのまま独立したユーザーデータ型として扱います。

```text
Goal
---
Result
---
Feedback
```

箱庭側では、これを次の3型へ変換します。

```text
<Action>Goal
<Action>Result
<Action>Feedback
```

通信には、Action 専用の次の3種類の PDU を使用します。

```text
<Action>ActionRequest
<Action>ActionResponse
<Action>ActionFeedback
```

ユーザーデータと Action 共通管理情報は分離します。

## 2. 生成例

例として、次の `Fibonacci.action` を扱います。

```text
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

ユーザーデータ型は次のように生成されます。

```text
FibonacciGoal
  int32 order

FibonacciResult
  int32[] sequence

FibonacciFeedback
  int32[] partial_sequence
```

## 3. Request PDU

Request PDU は、Goal の送信と Cancel 要求を扱います。

```text
FibonacciActionRequest
  ActionRequestHeader header
  FibonacciGoal goal
```

共通ヘッダ案は次のとおりです。

```text
ActionRequestHeader
  uint8  version
  uint8  request_kind
  uint8  reserved[2]
  uint8  goal_id[16]
```

`request_kind` は次を表します。

```text
GOAL   = 1
CANCEL = 2
```

Goal 送信時は `goal` を使用します。

```text
request_kind = GOAL
goal_id      = G1
goal         = user-defined goal
```

Cancel 時は同じ PDU を使い、`goal` 部分は参照しません。

```text
request_kind = CANCEL
goal_id      = G1
goal         = ignored
```

## 4. Response PDU

Response PDU は、Goal の受理・拒否、Cancel の受理・拒否、最終 Result を扱います。

```text
FibonacciActionResponse
  ActionResponseHeader header
  FibonacciResult result
```

共通ヘッダ案は次のとおりです。

```text
ActionResponseHeader
  uint8  version
  uint8  response_kind
  uint8  status
  uint8  reserved
  uint8  goal_id[16]
```

`response_kind` は、何に対する応答かを表します。

```text
GOAL_RESPONSE   = 1
CANCEL_RESPONSE = 2
RESULT          = 3
ERROR           = 4
```

`status` は応答結果または終端状態を表します。

```text
ACCEPTED  = 1
REJECTED  = 2
SUCCEEDED = 3
CANCELED  = 4
ABORTED   = 5
ERROR     = 6
```

代表的な組み合わせは次のとおりです。

```text
GOAL_RESPONSE   + ACCEPTED
GOAL_RESPONSE   + REJECTED
CANCEL_RESPONSE + ACCEPTED
CANCEL_RESPONSE + REJECTED
RESULT          + SUCCEEDED
RESULT          + CANCELED
RESULT          + ABORTED
```

Goal 応答と Cancel 応答では、`result` 部分は参照しません。最終 Result の場合のみ使用します。

## 5. Feedback PDU

Feedback PDU は、実行途中の通知を扱います。

```text
FibonacciActionFeedback
  ActionFeedbackHeader header
  FibonacciFeedback feedback
```

共通ヘッダ案は次のとおりです。

```text
ActionFeedbackHeader
  uint8  version
  uint8  reserved[3]
  uint8  goal_id[16]
  uint32 sequence_no
```

`sequence_no` は、同じ Feedback の再読、取りこぼし、順序の確認に利用します。

## 6. Goal ID

`goal_id` は1回の Action 実行セッションを識別します。

```text
goal_id
  Goal request
  Goal response
  Feedback 0..N
  Cancel request
  Cancel response
  Result
```

ROS 2 と接続する場合は、ROS Goal UUID をそのまま保持できるよう 128 bit とします。

箱庭ネイティブクライアントの場合は、クライアント側で同等の UUID を生成します。

同一 `goal_id` で別の Goal を再投入することは許可しません。

## 7. Client ID を持たない理由

Service RPC では、複数クライアントの要求を区別するために、次のような相関情報が必要でした。

```text
client_id + request_id
```

Action では、1つの Goal に対して1つのクライアントが対応し、Goal に関する全通信を `goal_id` で相関できます。

```text
Client A -> Goal G1
Client B -> Goal G2
```

したがって、Action PDU の意味論として `client_id` は持ちません。

Response や Feedback の配送に宛先情報が必要な場合、それは Action PDU ではなく Endpoint / Transport 層の責任とします。

## 8. Service との比較

| 観点 | Service | Action |
| --- | --- | --- |
| 実行単位 | Request | Goal |
| 相関情報 | client_id + request_id | goal_id |
| Client から Server | Request | Goal / Cancel |
| Server から Client | Response 1回 | Goal応答 / Feedback / Result |
| 中間通知 | なし | Feedback 0回以上 |
| 終端 | Response | Result |
| 状態管理 | 一往復 | Goal単位のライフサイクル |

Action は Service の Request / Response をそのまま拡張するのではなく、Goal を中心とした独立プロトコルとして扱います。

## 9. 基本シーケンス

正常終了:

```text
Request(GOAL, G1, Goal)
  -> Response(GOAL_RESPONSE, G1, ACCEPTED)
  <- Feedback(G1, sequence=1, Feedback)
  <- Feedback(G1, sequence=2, Feedback)
  <- Response(RESULT, G1, SUCCEEDED, Result)
```

Reject:

```text
Request(GOAL, G1, Goal)
  -> Response(GOAL_RESPONSE, G1, REJECTED)
```

Cancel:

```text
Request(CANCEL, G1)
  -> Response(CANCEL_RESPONSE, G1, ACCEPTED)
  -> Response(RESULT, G1, CANCELED, Result)
```

## 10. 今後の設計項目

このデータモデルを基に、次を別途決定します。

- 1 Action あたりの Request / Response / Feedback チャネル構成
- 同時 Goal 数と BUSY / Reject の扱い
- Feedback の保持方式
- タイムアウト
- Goal / Cancel / Result の競合
- PDU Registry の生成命名規則
- C++ / Python の生成対象
- ROS 2 内部通信とのマッピング
