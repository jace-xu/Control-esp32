# TaskCoordinator — 上层任务协调器（阶段调度 + 放置流程）

> 按「流程在 TaskCoordinator、其他类只提供动作」的架构：`ArmControl`/`ArmSequence`/`TrayControl`
> 只提供动作原语，**阶段编排与放置流程状态机全部放在本类**。
> 依赖注入：构造传入 *ArmControl* / *ArmSequence* / *TrayControl* 指针（不拥有其生命周期）。
> 持有 *ArmControl* 是因为放置流程要直接驱动臂电机（阶段1 仍由 *ArmSequence* 编排）。

## 职责
- **阶段1（A 键夹取入仓）**：转发给 [ArmSequence](../ArmSequence/README.md) 跑视觉对准+夹取+收臂；
  本类用 `arm_seq->isActive()` 的忙闲边沿编排物料盘入仓握手。
- **阶段2（X 键放置出料）**：**纯手动逐步确认**的放置流程状态机（取代原 *PlaceSequence*）。
  每按一次 X 推进一步（人确认上一步到位），**不做视觉对准、不做位置轮询自动判定**。
- 每帧推进物料盘 `tray->update(millis())`。
- 阶段1 与阶段2 **互斥运行**（同一 ArmControl，二选一）。

## 接口
- `TaskCoordinator(arm, armSeq, tray)` — 构造，注入三个子系统
- `update(a, x, b, l1, dpadX, dpadY)` — 每帧主入口
    - A：阶段1 夹取；X：阶段2 放置（启动 / 逐步推进）；B：中止；L1：阶段1 预备摆位
    - dpadX/dpadY：放置 `P_AIM` 手动瞄准（**左右→角度电机5，上下→前后电机7**）
- `isActive()` — `arm_seq 忙 || tray 忙 || 放置流程非 IDLE`
- `stop()` — 停 arm_seq + 停 tray + 放置流程复位 IDLE（中止/急停/手柄断连）

## 阶段1 入仓握手（count 来源）
靠 `arm_seq->isActive()` 边沿驱动物料盘存料，与阶段2 出料对称：
- 阶段1 **启动**（idle→active）→ `tray->prepareStore()`（顶料升起+挡片退位）
- 阶段1 **正常结束**（active→idle，非中止）→ `tray->store()`（落下+推进+**count+1**）
- 阶段1 **中止**（B）→ `tray->stop()` 取消入仓，count 不增

> `count` 是阶段2 `dispense()` 出料的前提：必须先经阶段1 入仓产生数量，否则放置启动会提示队空不动。

## 阶段2 放置流程状态机（X 逐步推进）
```
P_IDLE →(X,count>0) P_TO_PORT →(X) P_DISPENSE →(X) P_GRIP →(X) P_FWD_NEXT
→(X) P_ROTATE_DOWN →(X) P_AIM[手动d-pad] →(X) P_DESCEND_BOTTOM →(X) P_RELEASE
→(X) P_RETURN →(count>0) P_DISPENSE / (count==0) P_IDLE
```

| 状态 | X 触发时执行的动作 |
|------|------|
| `P_TO_PORT` | `arm->release()` 开爪 + `arm->setAnglePosition` 转到出料口；**首次**额外 `tray->compact()` 压紧 |
| `P_DISPENSE` | `tray->dispense()` 顶料舵机顶起一个物块 |
| `P_GRIP` | `arm->grip()` 合爪领取 |
| `P_FWD_NEXT` | `arm->setForwardPosition` r轴前伸到头 + `tray->notifyPicked()`（落下/送下一块/count−1） |
| `P_ROTATE_DOWN` | `arm->setAnglePosition` 转180° + `arm->setVerticalPosition` 下降一段 |
| `P_AIM` | **手动模式**：每帧 d-pad 位置点动瞄准；按 X 推进 |
| `P_DESCEND_BOTTOM` | `arm->setVerticalPosition` 下降到底 |
| `P_RELEASE` | `arm->release()` 开爪放置 |
| `P_RETURN` | 回升 + 角度回出料口；`count>0` 续 `P_DISPENSE`，否则 `P_IDLE` |

- **启动**：`P_IDLE` 且 `count>0` 时按 X → `P_TO_PORT`（队空打印提示不启动）
- **循环**：放完一个回 `P_DISPENSE`（compact 只首次做一次；后续物块已被 `notifyPicked` 送到出料口）
- **中止（B）**：`arm->release()` + `arm->stop()` + `tray->stop()` + 回 `P_IDLE`

### 手动瞄准（`P_AIM`，位置步进）
每帧按住 d-pad 即目标角 `+= kJogStepDeg`，用 `arm->setAngleForwardPosition(angle,forward)`
把角度(5)+前后(7) **打包成一条长命令一次性下发**（一帧只占一次总线，天然满足 2ms 间隔）。
方向反了改 `kJogAngleSign`/`kJogForwardSign` 符号。`angle_target`/`forward_target` 跟踪下发值，不读位置。

## 注意事项
- [ ! ] 所有位置/角度为占位常量（`kPortAngleDeg`/`kForwardEndDeg`/`kRotate180Deg`/`kAimDescendDeg`/
  `kBottomDeg`/`kReturnUpDeg`/`kJogStepDeg`），逻辑未实车验证，首跑前需现场校准（撞机/夹空风险）。
- [ ! ] 共享 Serial2 总线：同帧连发两条电机命令之间须留 ~2ms（`kBusDelayMs`），否则丢命令/读不到回复。
- [ ! ] 步进锁 `kStepLockMs`：步进后短时间内忽略 X，防手快误推进（电机没动就被推进）。
- 不新造电机底层函数，全部调用现有 *ArmControl* / *TrayControl* 接口。
- 拷贝构造与赋值已禁用。
