# ArmSequence — 机械臂高层任务编排（阶段1 夹取）

> 编排阶段1（A 键夹取 + 收臂归位），并把阶段2（X 键放置）委托给 [PlaceSequence](../PlaceSequence/README.md)。
> 视觉对准复用 [ArmControl](../ArmControl/README.md) 集成的对准接口（`beginAlign`/`updateAlign`/…），本类不再直接持有 *VisionSerial*/握手。
> 依赖注入：构造只传 *ArmControl* 指针。main 每帧调一次 `update()`。
> 占位参数见 [doc/ARM_TUNING.md](../../doc/ARM_TUNING.md)。

## 职责拆分（v3.0.0）
原来阶段1、阶段2 挤在一个 `runVerticalSequence` 状态机里靠 `mode` 分流；现已拆开：
- **视觉对准机制**（角度+前后 PID 闭环 + 串流握手 + 收敛判定）→ 合并进 **ArmControl**，两阶段共用。
- **阶段1（夹取 + 收臂）** → 留在本类。
- **阶段2（取料 + 折回 + 放置）** → 拆到 **PlaceSequence**（本类持有并在运行期委托）。
- **颜色队列**（`ColorQueue`）由本类拥有：阶段1 入队，注入 PlaceSequence 供阶段2 出队。

阶段1 与阶段2 互斥运行（`active` / `place.isActive()` 二选一），共用同一 ArmControl 无冲突。

## 阶段1（A 键，夹取 + 收臂归位）
- 先按 **L1**（可选）→ PRE_CATCH：角度电机预备摆位到 `kPreCatchAngleDeg`，再按 A 进对准（不按 L1 可直接 A 跳过）
- **第 1 次 A**：`arm->beginAlign(1,…)`（请求误差1）+ 先降一小段（`kPreDescendRotations`）+ 角度/前后 PID 对准
- **第 2 次 A**（已收敛 `arm->isAligned()`）：停视觉 → 位置降到夹取位（`kDescendTargetRotations`）→ 到位 `grip()` 夹紧
- **收臂归位**（夹紧 settle 后，多步，每步轮询到位）：
    1. 上下电机 + 前后电机同时归位（`kStowLiftRotations` / `kStowForwardDeg`）
    2. 角度电机归位（`kStowAngleDeg`）
    3. 上下电机再下降（`kFinalDescendRotations`）→ 到位 `release()` 放下物料 → 结束
- 握手成功记录一个物料颜色入队（每轮一个）
- **中止（B 键）**：丢弃本轮已记颜色 + `release()` 松爪，再停止复位

## 阶段2（X 键，放置）
见 [PlaceSequence](../PlaceSequence/README.md)。本类只在空闲时 X 上升沿调 `place.tryStart()` 启动，运行期把
`update(now, xRising, abort)` 全权委托给它；`isActive()` / `stop()` 并入 place 状态。

## 阶段1 状态机
`IDLE →PRE_CATCH →(A) PRE_ALIGN →(A,收敛) DESCENDING → GRIPPING → STOW_LIFT_FORWARD → STOW_ANGLE → STOW_DESCEND →(release) IDLE`

| 状态 | 说明 |
|------|------|
| `PRE_CATCH` | L1 进入，角度电机发一次位置命令摆位；不开视觉/不跑 PID；等 A；超时 `kPreCatchTimeoutMs` |
| `PRE_ALIGN` | 已先降一小段，角度+前后跑 PID 对准（`arm->updateAlign`）；等第二次 A；超时 `kPreAlignTimeoutMs` |
| `DESCENDING` | 位置控制，`readVerticalPosition` 轮询到位（阈值 `kPositionArrivedThreshDeg`）；到位 `grip()` |
| `GRIPPING` | 等 `kGripSettleMs` 后进收臂 |
| `STOW_LIFT_FORWARD` | 上下+前后同时归位，**两轴都轮询到位**才进下一步 |
| `STOW_ANGLE` | 角度电机归位，轮询到位 |
| `STOW_DESCEND` | 上下再下降，到位 `release()` 放料 → 结束 |

## 内部结构
- 启动：`startRecordSequence`（A）/ `startPreCatch`（L1）；阶段2 启动转发 `place.tryStart`（X）
- `enterPreAlign` — 先降一小段 + 进 PRE_ALIGN（IDLE+A 与 PRE_CATCH+A 复用）
- `confirmAlignAndDescend` — 第二次 A 确认下降到夹取位
- `runVisualServo` — 调 `arm->updateAlign`；用返回的 `freshFrame`/`color` 记色入队，`converged` 提示按 A
- `runVerticalSequence` — 阶段1 状态机；DESCENDING/STOW_* 均位置轮询到位
- `finishRun` — 阶段1 单轮完成收尾
- 停止原语：`haltMotors` / `resetRunState` / `haltAndReset`

## 注意事项
- [ ! ] 阶段2（取料折回放置）已在 PlaceSequence 实现，但占位参数需现场校准，逻辑未实车验证
- [ ! ] STOW/位置轮询需角度(5)/前后(7)电机驱动器开启"响应"，否则 `readPosition` 读不到、只能靠超时兜底
- [ ! ] 后续阶段2 可加第三次确认防止记录队列与真实不符，及松爪前的不一致校验（可移动底盘）
- [ ! ] 方向由 `kVerticalDirection` / `kRotationDirection` 因子统一控制，方向反了改因子符号
- [ ! ] 非串流期 DESCENDING/STOW_* 停一次伺服（`servo_stopped`）；PRE_CATCH/PRE_ALIGN 排除
- 占位值见 [doc/ARM_TUNING.md](../../doc/ARM_TUNING.md)；拷贝构造与赋值已禁用
