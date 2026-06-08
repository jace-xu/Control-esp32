# TaskCoordinator — 上层任务协调器（骨架）

> 未来承载**阶段调度 + 机械臂↔物料盘协同**：
> 阶段1 机械臂夹取→收臂→放入物料盘；中间物料盘存储；出料阶段物料盘送出→机械臂夹取→放置。
> 依赖注入：构造传入 *ArmSequence* 与 *TrayControl* 指针（不拥有其生命周期）。
> **[ ! ] 当前为透传层**：`update` 直接转发给 ArmSequence，无实际调度逻辑。

## 现状（透传）
- `TaskCoordinator(armSeq, tray)` — 构造，注入两个子系统
- `update(a, x, b, l1)` — 透传给 `ArmSequence::update`（行为与改动前一致）
- `isActive()` — 转发 `arm->isActive()`
- `stop()` — 转发 `arm->stop()`

main 现在通过本协调器驱动机械臂（`g_task_coordinator->update(...)`），不再直接调 ArmSequence。

## 待补（物料盘动作就位后）
- `update` 内按阶段插入物料盘调度：
    - 阶段1 机械臂放料到位 → `tray->store(color)`
    - 出料阶段 → `tray->dispense(color)`，到位后交机械臂夹取
    - 每帧推进 `tray->update(millis())`
- `isActive()` 并入物料盘忙状态（`|| tray->isBusy()`）
- `stop()` 加 `tray->stop()`
- 新增"中间存储阶段"的阶段机（机械臂阶段1 与阶段2 之间）

## 设计意图
把"阶段编排"从 ArmSequence 上移到本层，让 ArmSequence 逐步退化为**机械臂动作提供者**（夹取/放置/收臂），TrayControl 提供物料盘动作，本协调器负责按阶段串起两者。当前只先占住这个架构位置。
