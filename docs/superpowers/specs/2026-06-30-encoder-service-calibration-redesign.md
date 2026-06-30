# 磁编采样与标定重构设计

- 项目: MPS_MotorDriver
- 模块: MA600A 磁编码器 / FOC 角度链路 / 编码器标定
- 日期: 2026-06-30
- 状态: 已评审通过, 待实现

## 1. 背景

当前工程里 MA600A 同时存在调试轮询读取和 FOC ISR 内读取两条路径。实测出现磁编角度异常尖峰、角度非线性严重、标定结果不稳定。问题不应先归因于电机或磁钢本体, 需要先把软件采样链路收敛为单一事实源, 再做尖峰抑制、连续角维护和标定质量门控。

## 2. 目标

- 统一磁编读取入口, 避免 ISR 和 debug 命令并发访问 SPI/CS。
- 支持两种采样模式: ISR 同步采样、后台轮询采样。
- FOC 控制只消费稳定快照, 不直接触碰 MA600A 低层驱动。
- 对 raw angle 增加时间戳、状态、连续角、速度估计和异常计数。
- 对单点尖峰做隔离, 默认不让异常样本直接进入 FOC 角度。
- 标定流程使用同一份 encoder snapshot, 并输出可判定的质量指标。

## 3. 非目标

- 不在本轮引入 Stage 5 电流环控制逻辑。
- 不重写 SPI 外设初始化和板级引脚配置。
- 不改变 motor mode 的对外语义, 只替换角度来源实现。
- 不用滤波掩盖机械安装问题；软件先保证采样链路可信。

## 4. 核心判断

### 4.1 当前风险

1. debug 轮询读和 ISR 读如果同时发生, 会争用 SPI2/CS, 导致帧错位、读回瞬态异常或时序抖动。
2. ISR 内同步读直接把 raw 值用于角度链路, 缺少快照边界和异常隔离。
3. 标定逻辑若直接调用底层读角, 会与运行态读取模型不一致, 标定结果无法代表真实控制路径。
4. 异常尖峰若被累积进 unwrap/offset, 会放大为连续角跳变和非线性误判。

### 4.2 设计原则

- 单写多读: 只有 encoder service 负责更新编码器状态。
- 快照不可变: 控制、调试、标定都读取同一份 snapshot。
- 低层无策略: MA600A driver 只做寄存器/帧读取, 不做滤波和标定。
- 策略可观测: 所有丢样、尖峰、超时、校验失败都进入计数器。
- 标定可拒绝: 质量差时不给出“看似成功”的 offset/table。

## 5. 模块划分

| 模块 | 职责 |
| --- | --- |
| `ma600a.*` | MA600A 原始帧读取、寄存器访问、错误码返回 |
| `motor_encoder_at32m412.*` | SPI/CS 平台适配、临界区保护、硬件读写封装 |
| `encoder_service.*` | 采样调度、snapshot 更新、unwrap、速度估计、异常抑制 |
| `motor_calibration.*` | 使用 snapshot 做 offset/table 标定、质量评估 |
| `ma600a_debug.*` / shell | 只读 snapshot 或显式请求 service 读一次, 不直接抢 SPI |

## 6. 数据模型

```c
typedef struct {
    uint16_t raw;              /* 0..4095 */
    uint16_t raw_filtered;     /* spike gate 后的 raw */
    int32_t continuous_count;  /* 12-bit unwrap 后累计计数 */
    float mechanical_rad;
    float electrical_rad;
    float velocity_rad_s;
    uint32_t sample_tick;
    uint32_t age_us;
    uint32_t seq;
    uint32_t ok_count;
    uint32_t err_count;
    uint32_t spike_count;
    uint32_t stale_count;
    uint8_t valid;
    uint8_t stale;
    uint8_t spike_rejected;
} encoder_snapshot_t;
```

## 7. 采样模式

### 7.1 ISR 同步采样

- 在 FOC ISR 固定位置调用 `encoder_service_sample_isr()`。
- 函数内部做一次平台读角, 成功后更新 snapshot。
- 失败或尖峰时保持上一有效角度, 同时计数。
- FOC 获取 `encoder_service_get_snapshot()` 的快照副本。

### 7.2 后台轮询采样

- 由 shell/debug 或周期线程调用 `encoder_service_poll_once()`。
- 适用于非闭环控制、静态调试和标定前检查。
- 当 FOC 运行时默认禁止 debug 直接轮询底层 MA600A；如需读取, 只能读 snapshot。

### 7.3 模式切换

- `ENCODER_SAMPLE_ISR`: 闭环/开环控制默认模式。
- `ENCODER_SAMPLE_POLL`: 调试默认模式。
- 切换模式时 reset spike/unwrap 的短期状态, 保留累计诊断计数。

## 8. 尖峰与非线性处理

### 8.1 尖峰判定

- 先计算 12-bit wrap delta: `delta = wrap_i16(raw - last_raw, 4096)`。
- 根据采样周期和最大机械速度得到动态阈值。
- 初始保守阈值: 单 ISR tick 不允许超过 `ENCODER_MAX_DELTA_PER_TICK`。
- 超阈值样本标记为 spike, 不更新 `continuous_count` 和 FOC 角度。

### 8.2 连续角维护

- 只有通过校验的样本参与 unwrap。
- 连续角用 `int32_t continuous_count` 保存, 避免 float 累积误差。
- `mechanical_rad = continuous_count * 2π / 4096`。
- `electrical_rad = mechanical_rad * pole_pairs - offset + table_correction`。

### 8.3 非线性补偿

- 第一阶段只实现质量评估和线性 offset。
- 第二阶段引入 `encoder_lut[4096 or 256]` 分段补偿。
- LUT 必须有质量门控: 单调性、残差峰峰值、残差 RMS、样本覆盖率。

## 9. 标定流程

1. 停止 FOC 输出, 进入安全标定态。
2. reset encoder service 状态和诊断计数。
3. 正向低速拖动一圈, 采集 raw/electrical reference。
4. 反向低速拖动一圈, 采集同样数据。
5. 对比正反向残差, 判断磁钢偏心、反向间隙或采样尖峰。
6. 生成 offset, 可选生成 LUT。
7. 若质量指标不达标, 标定失败并保留旧参数。

## 10. Shell 命令

| 命令 | 作用 |
| --- | --- |
| `enc_status` | 输出 raw、continuous、elec、age、valid、err/spike/stale 计数 |
| `enc_mode isr|poll` | 切换采样模式 |
| `enc_sample [n]` | 非运行态主动采样 n 次 |
| `enc_reset` | 清空 service 状态 |
| `enc_calib status` | 查看标定质量和当前 offset/LUT 状态 |

## 11. 验收标准

- FOC 运行时 debug 不再直接访问 MA600A 底层 SPI。
- ISR 和 poll 两种模式都能更新同一类 snapshot。
- 注入单点 raw 尖峰时, FOC 角度保持上一有效值, `spike_count` 增加。
- MA600A 连续读失败时, snapshot 标记 stale, 控制层可进入故障或降级。
- 标定失败时不会覆盖旧 offset/table。
- shell 可观测 raw、连续角、age 和异常计数。

