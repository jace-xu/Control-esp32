# ArmSequence — 机械臂高层任务编排

> 组合视觉伺服 + 上下电机位置序列 + 夹爪控制。main 每帧只需调一次 `update()`。
> 依赖注入：构造传入已初始化的 *ArmControl* 与 *VisionSerial* 指针。
> 占位参数见 [doc/ARM_TUNING.md](../../doc/ARM_TUNING.md)。

## 两阶段 + 颜色队列
两阶段统一为**位置控制**：都走「先降一小段 + PID 对准 + 按键确认 + 位置下降/回升」。

- **阶段1（A 键，MODE_RECORD）—— 夹取 + 收臂归位**
    - 先按 **L1** → PRE_CATCH：角度电机预备摆位到 `kPreCatchAngleDeg`，再按 A 进对准（不按 L1 可直接 A 跳过）
    - **第 1 次 A**：先降一小段（`kPreDescendRotations`）+ 请求误差1 + 角度/前后 PID 对准
    - **第 2 次 A**（已收敛）：停视觉 → 位置降到夹取位（`kDescendTargetRotations`）→ 到位 `grip()` 夹紧
    - **收臂归位**（夹紧 settle 后，多步，每步轮询到位）：
        1. 上下电机 + 前后电机同时归位（`kStowLiftRotations` / `kStowForwardDeg`）
        2. 角度电机归位（`kStowAngleDeg`）
        3. 上下电机再下降（`kFinalDescendRotations`）→ 到位 `release()` 放下物料 → 结束
    - 握手成功记录一个物料颜色入队（每轮一个）
- **阶段2（X 键，MODE_CONSUME）—— 放置**
    - 从队列取一色 + 请求误差2 + 先降一小段 + PID 对准
    - **按 X 确认**（已收敛）：停视觉 → 位置降到放置位（`kPlaceDescendRotations`）→ 到位 `release()` 放料 → 等 `kGripDelayMs` → 回升
    - 自动取下一色直到队空（每色都需按 X 确认）
- **中止（B 键）**
    - 阶段1：丢弃本轮已记颜色 + `release()` 松爪；再停止复位
    - 阶段2：颜色队列保留

## 状态机
- 阶段1：`IDLE →PRE_CATCH →(A) PRE_ALIGN →(A,收敛) DESCENDING → GRIPPING → STOW_LIFT_FORWARD → STOW_ANGLE → STOW_DESCEND →(release) IDLE`
- 阶段2：`IDLE →(X) PRE_ALIGN →(X,收敛) DESCENDING → GRIPPING → ASCENDING → IDLE`

| 状态 | 说明 |
|------|------|
| `PRE_CATCH` | L1 进入，角度电机发一次位置命令摆位；不开视觉/不跑 PID；等 A；超时 `kPreCatchTimeoutMs` |
| `PRE_ALIGN` | 已先降一小段，角度+前后跑 PID 对准；等确认键（阶段1=A，阶段2=X）；超时 `kPreAlignTimeoutMs` |
| `DESCENDING` | 两阶段均位置控制，`readVerticalPosition` 轮询到位（阈值 `kPositionArrivedThreshDeg`）；到位 阶段1 `grip()` / 阶段2 `release()` |
| `GRIPPING` | 阶段1 等 `kGripSettleMs` 后进收臂；阶段2 等 `kGripDelayMs` 后回升 |
| `STOW_LIFT_FORWARD` | [阶段1] 上下+前后同时归位，**两轴都轮询到位**才进下一步 |
| `STOW_ANGLE` | [阶段1] 角度电机归位，轮询到位 |
| `STOW_DESCEND` | [阶段1] 上下再下降，到位 `release()` 放料 → 结束 |
| `ASCENDING` | [阶段2] 位置控制回升，位置轮询到位 |


## 内部结构
- 启动：`startRecordSequence`（A）/ `startPreCatch`（L1）/ `startConsumeStage`（X）
- `beginVerticalSequence` — 开视觉串流 + resetPID + `enterPreAlign`（两阶段统一）
- `enterPreAlign` — 先降一小段 + converge 清零 + 进 PRE_ALIGN（多处复用）
- `confirmAlignAndDescend` — 确认下降，按 mode 选夹取位/放置位
- `runVisualServo` — 仅收 `id1` 匹配帧；握手确认、阶段1 记色、PID 伺服、PRE_ALIGN 收敛计数
- `runVerticalSequence` — 状态机；DESCENDING/ASCENDING/STOW_* 均位置轮询到位（STOW_LIFT_FORWARD 同时轮询上下+前后两轴）
- `finishRun` — 跑完推进（阶段2 续下一色，否则结束）
- 停止原语：`haltMotors` / `resetRunState` / `haltAndReset`
- 颜色队列：`enqueueColor` / `dequeueColor` / `dropLastRecordedColor`

## 注意事项
- [ ! ] 仅写了放置时的状态机 未写从物料盘中夹取的状态
- [ ! ] 放置过程为根据pid调整前后与角度的电机后直接下降电机放置 未记录当前位置后折回再返回放置 
- [ ! ] 目前放置 可能由于多种问题导致无法正常运行 但不影响编译与夹取 在后续会补上
- [ ! ] 阶段1 夹紧后已加收臂归位（上下+前后→角度→上下再降→松爪），目标为占位常量需现场校准
- [ ! ] [ ! ] 阶段1 结尾会 `release()` 松爪放下物料——阶段1 已是自包含的夹取→摆位→放下；与阶段2 放置的关系待重新厘清
- [ ! ] STOW/位置轮询需角度(5)/前后(7)电机驱动器开启"响应"，否则 `readPosition` 读不到、只能靠超时兜底
- [ ! ] 后续阶段二应加上第三次确认防止因某些原因记录的队列与真实情况不符 及在松开舵机前加一个x键确认若不一致可移动底盘控制
- [ ! ] 方向由 `kVerticalDirection` / `kRotationDirection` 因子统一控制，方向反了改因子符号
- [ ! ] 非串流期 DESCENDING/ASCENDING/STOW_* 停一次伺服（`servo_stopped`）；PRE_CATCH/PRE_ALIGN 排除，否则 `stopServo` 会打断角度电机位置移动

- 占位值见 [doc/ARM_TUNING.md](../../doc/ARM_TUNING.md)；拷贝构造与赋值已禁用
