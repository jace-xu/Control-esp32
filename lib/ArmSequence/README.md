# 2026.06.04

## 这是机械臂高层任务编排类，组合视觉伺服 + 上下电机自动序列
- 把原 main.cpp 中的机械臂状态机和相关状态收归自身，main 每帧只需调用一次 `update()`
- 依赖注入：构造时传入已初始化好的 *ArmControl* 与 *VisionSerial* 指针
    - [ ! ] 本类不拥有它们的生命周期（不负责 delete）
- 目前只写了夹取和放置两个阶段 + 颜色队列模式：A 键记录物料颜色入队，X 键按队列逐个消费，B 键强制打断停止

## 两阶段任务模式
- **阶段1（A 键，MODE_RECORD）**
    - 请求误差1，握手成功后记录一个物料颜色入队
    - 跑完一整套升降序列，每按一次 A 记一个颜色
- **阶段2（X 键，MODE_CONSUME）**
    - 请求误差2（按颜色），从队列 FIFO 逐个取色
    - 自动连续跑完队列里所有颜色的升降序列，直到队空
- **中止（B 键）**
    - 立即停止所有动作并复位为 IDLE，颜色队列保留

## 上下电机状态机 
- `IDLE → DESCENDING → GRIPPING → ASCENDING → IDLE`
    - IDLE：空闲
    - DESCENDING：下降中（开环时间控制，预估 6 秒，超时保护 15 秒）
    - GRIPPING：[!]到底后暂停等待夹取（现定 5 秒之后要加入舵机控制夹爪夹取 等待夹取结束后继续下一步）
    - ASCENDING：回升中（预估 5 秒，超时保护 12 秒）
- [ ! ] 当前全部用时间估算；位置控制协议实现后可替换

## 公共接口
- `ArmSequence(arm, vision)`
    - 构造，注入机械臂控制与视觉串口对象指针
- `update(aPressed, xPressed, abortPressed)`
    - 每帧主入口，转发 A/X/B 三键状态
    - A/X 上升沿在空闲时触发对应模式；序列自锁运行（松开按键不影响）
    - 运行期间按 B 立即停止
- `isActive()`
    - 视觉伺服 / 序列是否正在运行，供 main 做安全判断
- `queuedColorCount()`
    - 当前颜色队列中待抓物料数量
- `stop()`
    - 立即停止所有电机并复位状态机（= 内部 `haltAndReset()`）
    - 中止 / 紧急停止 / 手柄断连超时时调用；颜色队列保留
- `clearColorQueue()`
    - 手动清空颜色队列

## 停止
- `haltMotors()`
    - 停串流 + 停三个电机并复位 PID（只碰硬件，不动状态机）
    - 供 `finishRun()` 跑完一轮回静止时复用
- `resetRunState()`
    - 复位状态机 + 解除自锁（只碰标志，不碰电机/队列）
- `haltAndReset()`
    - 上面两个的组合，即“彻底停止”
    - `stop()` 与超时保护共用此路径

## 内部序列流程
- `startRecordSequence(t)` — 阶段1启动：请求误差1，进入升降序列
- `startConsumeStage(t)` — 阶段2启动：从队列取首色，队空则不启动
- `beginVerticalSequence(t, errorType, color)`
    - 两阶段共用：上锁 → 请求对应误差串流 → resetPID → 进入 DESCENDING 开始下降
- `runVisualServo(t)`
    - 读取视觉误差，**仅接受 `id1` 与本次请求误差类型匹配的帧**
    - 切换串流瞬间残留的旧类型帧会被滤掉，防止误判握手 / 记错颜色 / 用旧误差驱动 PID
    - 匹配帧：确认握手、阶段1记录颜色、执行 PID 伺服
    - 无匹配帧：停伺服并复位 PID，不中止序列（升降照常推进）
- `runVerticalSequence(t)`
    - 升降状态机推进，含下降 / 回升超时保护（超时走 `haltAndReset()`）
- `finishRun(t)`
    - 正常跑完一轮的推进逻辑
    - 阶段2队列还有颜色 → 自动启动下一颜色（自锁继续）
    - 阶段1单轮 / 阶段2队空 → `resetRunState()` 解锁结束

## 视觉串流握手
- `startVisionStream(t, errorType, color)`
    - 请求树莓派发误差数据（幂等，已串流则不重发）
- `sendStartCommand()` — 按当前误差类型发对应 START 命令
- `stopVisionStream()` — 通知树莓派停止发送（幂等）
- `retryStartIfNeeded(t)`
    - START 帧可能被干扰丢失，收到首帧前每 300ms 重发，最多 10 次（约 3 秒）

## 颜色队列（FIFO 环形缓冲）
- `enqueueColor(color)` — 入队，队满（容量 16）丢弃并告警
- `dequeueColor(color)` — 出队取队首，队空返回 false

## 注意事项
- [ ! ] 不负责 *ArmControl* / *VisionSerial* 的创建与销毁
- [ ! ] 升降为开环时间控制，机械结构 / 负载变化时需重新整定时间常量
- [ ! ] 中止与超时不清空颜色队列，需要时手动调 `clearColorQueue()`
- 拷贝构造与赋值已禁用
