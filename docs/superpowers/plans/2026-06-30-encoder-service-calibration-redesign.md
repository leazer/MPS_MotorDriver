# 磁编采样与标定重构实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` or `superpowers:executing-plans` to implement task-by-task. Keep Stage 5 current-loop work out of this branch.

**Goal:** 重构 MA600A 磁编码器链路, 建立统一 encoder service, 同时支持 ISR 同步采样和后台轮询采样, 修复 debug/ISR 争用、异常尖峰传播和标定质量不可控的问题。

**Spec:** `docs/superpowers/specs/2026-06-30-encoder-service-calibration-redesign.md`

---

## File Structure

| 文件 | 操作 | 职责 |
| --- | --- | --- |
| `application/motor_control/encoder_service.h` | 创建 | snapshot 类型、模式枚举、service API |
| `application/motor_control/encoder_service.c` | 创建 | 采样调度、unwrap、尖峰拒绝、速度估计 |
| `application/motor_control/ma600a.h` | 修改 | 明确 raw read API 和错误码语义 |
| `application/motor_control/ma600a.c` | 修改 | 保持低层读帧, 移除策略性状态 |
| `application/motor_control/motor_encoder_at32m412.*` | 修改 | SPI/CS 平台适配和临界区保护 |
| `application/motor_control/motor_control_isr.c` | 修改 | FOC 只消费 encoder snapshot |
| `application/motor_control/motor_calibration.*` | 修改 | 标定改为读取 snapshot, 增加质量门控 |
| `application/motor_control/ma600a_debug.*` / shell | 修改 | debug 读 snapshot, 不抢底层 SPI |
| `tests/motor_control/test_encoder_service.c` | 创建 | host 侧验证 unwrap、尖峰拒绝、stale |

---

## Task 1: 建立 encoder service 骨架

- [ ] 新增 `encoder_service.h/.c`, 定义 `encoder_snapshot_t`、采样模式、错误码和 public API。
- [ ] API 至少包含 `init/reset/set_mode/sample_isr/poll_once/get_snapshot/get_stats`。
- [ ] 暂时用可注入 raw provider, 方便 host 测试不依赖硬件。
- [ ] 新增 host 单测文件, 先验证 reset、snapshot seq、valid 状态。

## Task 2: 收口 MA600A 原始读取路径

- [ ] 梳理所有 `ma600a_read*` 调用点。
- [ ] 底层 driver 只暴露“读 raw angle + 状态”的最小 API。
- [ ] 在平台适配层集中处理 CS 拉低/拉高、SPI busy、临界区保护。
- [ ] 保证 debug 模块不再绕过 service 直接访问 SPI。

## Task 3: 实现 unwrap 与尖峰拒绝

- [ ] 实现 12-bit wrap delta 计算。
- [ ] 用通过校验的 raw 更新 `continuous_count`。
- [ ] 超阈值 raw 标记 `spike_rejected`, 保持上一有效角。
- [ ] 单测覆盖正常跨 0、反向跨 0、单点尖峰、连续尖峰。

## Task 4: 接入 ISR 角度链路

- [ ] 在 FOC ISR 固定点调用 `encoder_service_sample_isr()`。
- [ ] FOC 电角度只从 snapshot 读取。
- [ ] snapshot stale 或 invalid 时使用现有故障/降级路径。
- [ ] 保持 open-loop/ramp 调试路径可用。

## Task 5: 接入 debug 与 shell

- [ ] `ma600a_debug` 默认展示 snapshot, 不在 FOC 运行态主动读底层 SPI。
- [ ] 增加 `enc_status` 输出 raw、continuous、electrical、age、err/spike/stale。
- [ ] 增加 `enc_mode isr|poll` 与 `enc_sample [n]`。
- [ ] 明确运行态下 `enc_sample` 的限制或拒绝提示。

## Task 6: 重构标定数据来源

- [ ] `motor_calibration` 改为消费 encoder snapshot。
- [ ] 标定前 reset service 短期状态。
- [ ] 标定过程中记录 raw、continuous、电角度参考、方向和时间戳。
- [ ] 标定读数异常时终止并保留旧参数。

## Task 7: 增加标定质量门控

- [ ] 输出 offset、残差 RMS、残差峰峰值、样本覆盖率、正反向一致性。
- [ ] 设置初始 fail 阈值, 不达标时拒绝写入新 offset/table。
- [ ] shell 输出清楚区分 pass、fail、沿用旧参数。
- [ ] 文档记录台架调参项和观测方法。

## Task 8: 验证与移交

- [ ] 运行 encoder service host 单测。
- [ ] 构建固件, 确认新增模块纳入 CMake/Keil 工程。
- [ ] 用 `enc_status` 观察静态 raw jitter、旋转连续性、spike/stale 计数。
- [ ] 更新 `doc/调试记录.md`, 记录磁编尖峰/非线性处理结果。

---

## Implementation Notes

- 优先保证采样链路确定性, 不急着做 LUT 补偿。
- 所有异常样本必须可观测, 不允许静默吞掉。
- 标定结果写入前必须经过质量判断。
- Stage 5 当前环相关改动不得混入本分支。

