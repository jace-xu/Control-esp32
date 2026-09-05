# PlaceSequence — 阶段2 放置序列（已退役，被 TaskCoordinator 取代）

> **[ ! ] 本模块已退役。** 阶段2 放置流程自 v3.2.2 起重写为**纯手动 X 键逐步确认**并移入
> [TaskCoordinator](../TaskCoordinator/README.md)：删除了视觉对准与颜色队列，瞄准改为手柄 d-pad 位置点动。
> 本类已不再被任何代码 include/使用，磁盘文件保留仅作历史参考，可后续删除。
> 下面是退役前（基于视觉+颜色队列的自动流程）的原始说明。

---

# PlaceSequence — 阶段2 放置序列（从物料盘取料 → 折回 → 放置）

> X 键触发，按颜色队列 FIFO 逐个消费。视觉对准复用 [ArmControl](../ArmControl/README.md) 集成的对准接口；
> 本类只负责阶段2 的步骤编排与位置轮询。
> 依赖注入：构造传入 *ArmControl* 与 *ColorQueue* 指针（颜色队列由 [ArmSequence](../ArmSequence/README.md) 拥有，阶段1 入队、本类出队）。
> 占位参数现场校准。

## 由来
原阶段2 是「先降 → 放料 → 回升」的简单流程，挤在 ArmSequence 里靠 `mode` 分流。现扩展为完整的
**从物料盘取料再放置**流程（约 8 个状态），故拆成独立状态机，与阶段1 互斥运行。

## 流程
1. 按 **X**（空闲）→ 从颜色队列取队首色 → `arm->beginAlign(2,color)`（请求误差2）+ 先降一小段 + PID 对准
2. **收敛后自动**（不等 X）：读 `readForwardPosition`/`readAnglePosition` 记录当前前后+角度位置（= 折回目标）
3. 移动角度+前后电机到**固定取料位**（`kPickupAngleDeg`/`kPickupForwardDeg`），轮询两轴到位
4. 下降上下电机到取料深度（`kPickupDescendRotations`）→ 到位 `grip()` 夹取
5. 等 `kGripSettleMs` settle → 上升上下（`kReturnAscendRotations`）+ 前后/角度**折回到第2步记录位**，轮询三轴到位
6. **按 X**（人工确认门）→ 下降上下到放置位（`kPlaceDescendRotations`）
7. **再按 X**（人工确认门）→ `release()` 松爪 → 本色完成
8. 队列非空 → 自动取下一色重跑；队空 → 结束
- **中止（B 键）**：`release()` 松爪 + 停止复位（颜色队列保留；已出队的本色视作丢失）

两个 X 均为**纯人工确认门**：按下直接推进，不重跑视觉对准。

## 状态机
`IDLE →(X) PRE_ALIGN →(收敛,自动) MOVE_TO_PICKUP → DESCEND_PICKUP → GRIP_SETTLE → ASCEND_RETURN →(X) DESCEND_PLACE →(X) [release] →下一色/IDLE`

| 状态 | 说明 |
|------|------|
| `PRE_ALIGN` | 先降一小段 + `arm->updateAlign` 跑 PID 对准；收敛即自动记位并推进；超时 `kPreAlignTimeoutMs` |
| `MOVE_TO_PICKUP` | 角度+前后 → 固定取料位，两轴轮询到位；超时 `kAscentTimeoutMs` |
| `DESCEND_PICKUP` | 上下 → 取料深度，到位 `grip()`；超时 `kDescentTimeoutMs` |
| `GRIP_SETTLE` | 等 `kGripSettleMs` 后上升并折回 |
| `ASCEND_RETURN` | 上下回升 + 前后/角度折回记录位，**三轴**轮询到位；超时 `kAscentTimeoutMs` |
| `WAIT_X_DOWN` | 人工确认门，等 X → 下降放置位；不超时（等人） |
| `DESCEND_PLACE` | 上下 → 放置位，轮询到位；超时 `kDescentTimeoutMs` |
| `WAIT_X_RELEASE` | 人工确认门，等 X → `release()` → 本色完成；不超时（等人） |

## 接口
- `PlaceSequence(arm, queue)` — 构造，注入机械臂与颜色队列
- `bool isActive()` — 是否正在运行
- `bool tryStart(now)` — X 上升沿启动（队空返回 false）
- `void update(now, xRising, abort)` — 每帧推进（仅 active 时由 ArmSequence 调用）
- `void stop()` — 立即停止并复位

## 内部结构
- `begin` — 开对准 + 先降一小段 + 进 PRE_ALIGN（启动与续下一色复用）
- `runAlign` — 调 `arm->updateAlign`；收敛瞬间记录对准位、停视觉、转位置控制进 MOVE_TO_PICKUP
- `runStateMachine` — 位置轮询 + 超时保护；`forwardArrived`/`angleArrived`/`verticalArrived` 到位辅助
- `finishRun` — 本色完成；队列非空续 `begin`，否则结束
- 停止原语：`haltMotors` / `resetRunState` / `haltAndReset`

## 注意事项
- [ ! ] 所有取料/折回/放置位均为占位常量，需现场校准；逻辑未实车验证
- [ ! ] 位置轮询需角度(5)/前后(7)电机驱动器开启"响应"，否则 `readPosition` 读不到、只能靠超时兜底
- [ ! ] `setCountConvergence(false)`：对准完成转位置控制后关闭收敛累计，避免位置态误判收敛
- [ ! ] 方向由 `kVerticalDirection` / `kRotationDirection` 因子统一控制
- [ ! ] 后续可加第三次确认防止记录队列与真实不符，及松爪前的不一致校验（可移动底盘）
- 拷贝构造与赋值已禁用