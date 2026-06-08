# VisionStreamHandshake — 视觉串流握手会话

> 管理与树莓派的视觉误差串流"会话状态":请求/停止串流 + START 命令超时重传。
> 从 ArmSequence 抽出,让机械臂状态机不再直接持有串流细节。

## 接口

构造:`VisionStreamHandshake(VisionSerial* vision)`(依赖注入,不拥有生命周期)

| 方法 | 作用 |
|---|---|
| `start(errorType, color, now)` | 请求串流(幂等:已串流则不重发)。errorType: 1=误差1, 2=误差2;color 仅误差2 用 |
| `stop()` | 通知树莓派停止(幂等:未串流则不动) |
| `retryIfNeeded(now)` | 未收到首帧时按间隔重传 START;每帧调用 |
| `confirm()` | 收到匹配帧时调,标记握手成功(停止重传) |
| `isStreaming()` | 是否已请求串流(供调用方判断是否进伺服分支) |
| `isConfirmed()` | 是否已收到首帧确认 |
| `expectedErrorType()` | 本次请求的误差类型(1/2),供帧类型校验 |


## 可调参数

| 常量 | 当前值 | 含义 |
|---|---|---|
| `kStartRetryIntervalMs` | 300 | 未收到数据则每 300ms 重发一次 START |
| `kStartMaxRetries` | 10 | START 最大重传次数(约 3 秒后放弃,打印警告) |

> 重传耗尽**不中止序列**:仅打印 "running blind" 警告,机械臂继续按时间序列动作(可能盲抓)。
