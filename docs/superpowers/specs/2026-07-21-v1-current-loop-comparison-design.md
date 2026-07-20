# V1 旧硬件电流环对比分支设计

## 目标

从硬件调整前的 `main@89b29a8` 建立 `compare/v1-current-loop`，移植 `feat/current-sampling-reconstruction@694e644` 已验证的电流环、采样重构、保护、诊断和台架脚本，同时保留 V1 板卡的 PWM 相序与 LED 引脚。该分支用于和 V2 板卡运行结果做 A/B 对比，不直接替代尚未完成台架验证的 `main`。

## 已确认的分支边界

- `main@89b29a8`：V1 硬件调整前基线。
- `9519e6f`：V2 BSP 映射提交，包含 U/W PWM 通道对调、LED 从 PA0 移到 PB8及其静态测试。
- `hw/v2-coaxial-encoder@1332944`：V2 硬件开发线。
- `feat/current-sampling-reconstruction@694e644`：继承 V2 映射并完成低侧采样极性修复和 ±500mA 八点验证的源分支。

## 方案比较

### 方案 A：直接把电流环分支合入 `main`

优点是操作最少。缺点是会把 `9519e6f` 的 V2 PWM/LED 映射一起带入旧板，旧板存在相序错误和调试指示错误风险，因此不采用。

### 方案 B：逐提交 cherry-pick，跳过 `9519e6f`

优点是保留提交历史。缺点是后续 PWM 发布和采样对齐提交依赖 `9519e6f` 引入的通道宏与测试文件，跳过后会产生跨提交冲突，容易遗漏最终树中的保护和诊断修正，因此不作为首选。

### 方案 C：在 V1 基线上做受控 squash 移植，再显式固定 V1 映射

先将源分支最终树以 squash 方式应用到独立对比分支，再把唯一的硬件差异边界改为 V1，并用静态测试锁定。该方案能最大限度保持两套控制软件一致，同时让 V1/V2 的运行差异集中在 BSP 映射，最适合台架 A/B 对比，因此采用本方案。

## 目标树设计

### 控制层保持一致

以下行为与 V2 已验证源分支保持一致：

- `current_reconstruction_run()` 的低侧采样窗口判定、择优两相和 KCL 补相。
- `raw_ia/raw_ib/raw_ic` 保留 MP6540H 低边器件电流极性；corrected `ia/ib/ic` 在进入保护和 Clarke/Park 前统一取反。
- PWM 请求在更新边界发布，采样计划与实际生效 PWM 周期配对。
- 无效帧连续计数、PI freeze、`FAULT_CURRENT_SAMPLE` 锁存和停机行为。
- `mc_debug`、`pwm_info`、`mc_cur`、`stage5_bench.py` 的诊断与验收语义。

### V1 BSP 映射保持不变

`board_motor_pins.h` 必须明确给出下列 V1 映射：

```c
#define PWM_PHASE_U_TMR_CHANNEL  TMR_SELECT_CHANNEL_1
#define PWM_PHASE_V_TMR_CHANNEL  TMR_SELECT_CHANNEL_2
#define PWM_PHASE_W_TMR_CHANNEL  TMR_SELECT_CHANNEL_3

#define LED_GPIO_PORT            GPIOA
#define LED_PIN                  GPIO_PINS_0
```

`project/inc/at32m412_416_wk_config.h` 的 LED 也保持 `GPIOA/GPIO_PINS_0`。PWM 驱动继续通过 `PWM_PHASE_*_TMR_CHANNEL` 宏写 CCR，使控制层无需包含板卡版本分支。

### V1 映射回归测试

将 `tests/test_pwm_mapping_static.py` 改为 V1 对比分支契约：

- U/V/W 分别断言 CH1/CH2/CH3。
- LED 断言 PA0。
- PWM 初始化和运行写入必须使用 `PWM_PHASE_*_TMR_CHANNEL`，禁止重新写死通道。
- 板级初始化通过端口宏配置 LED 和 SPI2 CS；两者同在 GPIOA 不影响独立初始化的功能正确性。

## 移植与提交策略

1. 从 `main@89b29a8` 创建 `compare/v1-current-loop` 独立 worktree。
2. 对 `feat/current-sampling-reconstruction@694e644` 执行 `git merge --squash`，不产生源分支祖先关系。
3. 在暂存前修改 V1 PWM/LED 映射和对应静态测试，并检查不含 PB8、U=CH3、W=CH1 的 V2 运行配置。
4. 以单个可审计提交落地，提交说明记录源分支和源提交。
5. 在合并结果上运行完整静态测试、严格主机 C 测试、ARM CMake 和 Keil clean build。
6. 软件验证通过后保留 `compare/v1-current-loop`，不自动快进 `main`；V1 实机八点矩阵通过后再决定是否更新 `main`。

## 台架 A/B 规则

V1 与 V2 使用同一套条件：母线约 12V、实验电源限流 1A、同一电机、同一编码器标定流程、CCR4=5264，以及 `Iq_ref=±50/±100/±200/±500mA` 八点矩阵。每个点记录 Iq、Id、valid mask、recon、invalid/freeze 和 fault；结束必须记录 `DISABLED`、EN=0、三相 50% 占空比和 fault=0。

V2 的既有 `tests/stage5_bench_log.txt` 只能作为 V2 证据。V1 分支在真实旧板执行前不得复制或宣称该数据为 V1 结果，实际输出应另存为 V1 日志，供论坛文章逐项对照。

## 风险与停止条件

- 如果 squash 后除明确的 PWM/LED 映射外仍存在板卡专属差异，停止提交并逐项审查。
- 如果 V1 映射静态测试、主机测试或任一目标工具链构建失败，不进入烧录。
- 软件构建通过不代表旧板台架通过；V1 首次上板仍从停机、零偏和 `mc_open 500 0` 极性门禁开始。
- 当前验证上限仍为 ±500mA，不测试 ±1.5A，也不把速度环纳入本轮对比结论。

## 验收标准

- `compare/v1-current-loop` 基于 `main@89b29a8`，工作树干净。
- 控制层与源分支的功能差异仅来自 V1 BSP 映射和 V1/V2 证据文档隔离。
- 所有静态测试和三项严格主机 C 测试通过。
- ARM CMake 构建成功；Keil clean build 为 0 Error。
- V1 映射断言 U/V/W=CH1/2/3、LED=PA0 全部通过。
- 在用户接入旧板后，再以独立日志完成八点电流矩阵和最终安全状态验证。
