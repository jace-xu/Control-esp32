# ArmSequence — 机械臂高层任务编排（阶段1 夹取）

> 编排阶段1（A 键夹取 + 收臂归位）。阶段2（X 键放置）已移到 [TaskCoordinator](../TaskCoordinator/README.md)，
> 本类不再持有 *PlaceSequence* / *ColorQueue*，也不再处理 X 键。
> 视觉对准复用 [ArmControl](../ArmControl/README.md) 集成的对准接口（`beginAlign`/`updateAlign`/…），本类不再直接持有 *VisionSerial*/握手。
> 依赖注入：构造只传 *ArmControl* 指针。main/TaskCoordinator 每帧调一次 `update(a,x,b,l1)`（x 已忽略）。
> 占位参数见 [doc/ARM_TUNING.md](../../doc/ARM_TUNING.md)。

## 职责拆分（v4.0.0）
逐步把「阶段编排」上移到 TaskCoordinator，让本类退化为**机械臂阶段1 动作提供者**：
- **视觉对准机制**（角度+前后 PID 闭环 + 串流握手 + 收敛判定）→ 在 **ArmControl**，两阶段共用。
- **阶段1（夹取 + 收臂）** → 留在本类。
- **阶段2（取料 + 放置）** → 移到 **TaskCoordinator**（纯手动 X 键逐步确认，不再用视觉/颜色队列）。
- **颜色队列**（`ColorQueue`）已移除：阶段2 不再按颜色出料，本类不再记色。

阶段1 与阶段2 互斥运行（由 TaskCoordinator 路由），共用同一 ArmControl 无冲突。

## 阶段1（A 键，夹取 + 收臂归位）
- 先按 **L1**（可选）→ PRE_CATCH：角度+前后电机预备摆位（`kPreCatchAngleDeg`/`kPreCatchForwardDeg`，
  `setAngleForwardPosition` 打包一次发出），再按 A 进对准（不按 L1 可直接 A 跳过）
- **第 1 次 A**：`arm->beginAlign(1,…)`（请求误差1）+ 先降一小段（`kPreDescendRotations`）+ 角度/前后 PID 对准
- **第 2 次 A**（已收敛 `arm->isAligned()`）：停视觉 → 位置降到夹取位（`kDescendTargetRotations`）→ 到位 `grip()` 夹紧
- **收臂归位**（夹紧 settle 后，多步，每步轮询到位）：
    1. 上下→0 + 前后→0 同时归位（`kStowVerticalHomeDeg` / `kStowForwardHomeDeg`，`setVerticalForwardPosition` 打包发出）
    2. 角度电机→0 归位（`kStowAngleDeg`）
    3. 前后电机伸出到放料位（`kStowForwardExtendDeg`）→ 到位 `release()` 放下物料 → 结束
- **中止（B 键）**：`release()` 松爪，再停止复位
- 入仓计数由 [TaskCoordinator](../TaskCoordinator/README.md) 用 `isActive()` 边沿编排（启动 `prepareStore`、结束 `store`）

## 阶段2（X 键，放置）
见 [TaskCoordinator](../TaskCoordinator/README.md)。本类不再参与；`update` 的 x 参数已忽略。

## 阶段1 状态机
`IDLE →PRE_CATCH →(A) PRE_ALIGN →(A,收敛) DESCENDING → GRIPPING → STOW_LIFT_FORWARD → STOW_ANGLE → STOW_FORWARD_EXTEND →(release) IDLE`

| 状态 | 说明 |
|------|------|
| `PRE_CATCH` | L1 进入，角度+前后发一次位置命令摆位；不开视觉/不跑 PID；等 A；超时 `kPreCatchTimeoutMs` |
| `PRE_ALIGN` | 已先降一小段，角度+前后跑 PID 对准（`arm->updateAlign`）；等第二次 A；超时 `kPreAlignTimeoutMs` |
| `DESCENDING` | 位置控制，`readVerticalPosition` 轮询到位（阈值 `kPositionArrivedThreshDeg`）；到位 `grip()` |
| `GRIPPING` | 等 `kGripSettleMs` 后进收臂 |
| `STOW_LIFT_FORWARD` | 上下→0 + 前后→0 同时归位，**两轴都轮询到位**才进下一步 |
| `STOW_ANGLE` | 角度电机→0 归位，轮询到位 |
| `STOW_FORWARD_EXTEND` | 前后电机伸出到放料位，到位 `release()` 放料 → 结束 |

## 内部结构
- 启动：`startRecordSequence`（A）/ `startPreCatch`（L1）
- `enterPreAlign` — 先降一小段 + 进 PRE_ALIGN（IDLE+A 与 PRE_CATCH+A 复用）
- `confirmAlignAndDescend` — 第二次 A 确认下降到夹取位
- `runVisualServo` — 调 `arm->updateAlign`；`converged` 提示按 A（不再记色）
- `runVerticalSequence` — 阶段1 状态机；DESCENDING/STOW_* 均位置轮询到位
- `finishRun` — 阶段1 单轮完成收尾
- 停止原语：`haltMotors` / `resetRunState` / `haltAndReset`

## 注意事项
- [ ! ] STOW/位置轮询需角度(5)/前后(7)电机驱动器开启"响应"，否则 `readPosition` 读不到、只能靠超时兜底
- [ ! ] 方向由 `kVerticalDirection` / `kRotationDirection` 因子统一控制，方向反了改因子符号
- [ ! ] 非串流期 DESCENDING/STOW_* 停一次伺服（`servo_stopped`）；PRE_CATCH/PRE_ALIGN 排除
- 占位值见 [doc/ARM_TUNING.md](../../doc/ARM_TUNING.md)；拷贝构造与赋值已禁用
