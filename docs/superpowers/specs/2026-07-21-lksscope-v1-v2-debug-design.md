# V1/V2 LKS Scope 电机调试视图设计

## 目标

为 V1、V2 电流环固件提供语义一致的 `debug.lksscope`：曲线直接显示编码器原始值、标定前/后电角度、DQ/三相电流以及电流环三相输出电压；数值列表覆盖日常定位状态机、传感器、采样重构、保护和控制环问题所需的变量。

## 已确认的信号语义

- 编码器原始值：`s_snapshot.raw16`，范围 0..65535。
- 标定前电角度：原始编码器值扣除电角度零点、乘极对数并归一化到 0..2π，单位 mrad；不应用 256 点非线性误差表。
- 标定后电角度：`s_snapshot.elec_mrad`，先应用编码器非线性误差表，再扣除零点并乘极对数，单位 mrad。
- `iu/iv/iw`：进入 Clarke/Park 的 FOC 符号约定三相电流，对应代码中的 corrected `ia/ib/ic`，单位 mA。
- `id/iq`：Park 变换后的实测 DQ 电流，单位 mA。
- `uu/uv/uw`：电流环输出 `vd/vq` 经逆 Park、逆 Clarke 后得到的三相电压指令，单位 mV；不包含 SVPWM 零序注入，也不是硬件实测端电压。

## 采用方案

采用最小调试接口扩展，不重构现有 ISR 调试快照：

1. 在 `encoder_snapshot_t` 增加 `raw_elec_mrad`，采样接受时同时发布标定前和标定后电角度。
2. 在 `foc_core` 增加纯函数 `foc_inv_clarke()`，由 SVPWM 和调试发布共用同一组相电压换算公式。
3. 在 CURRENT、SPEED 分支发布 `s_dbg_uu_mv/s_dbg_uv_mv/s_dbg_uw_mv`；非闭环状态清零，避免界面保留陈旧值。
4. 在 `motor_control_isr_debug_t` 暴露三相输出电压，保持 shell/其他调试消费者可用。
5. 基于用户当前主工作区文件保留 COM9、J-Link、采样周期及界面布局，只替换数值变量和曲线内容；主工作区的脏文件本身不被覆盖。
6. V1、V2 各自保存相同结构的 `debug.lksscope`，继续使用相对 AXF 路径 `./project/MDK_V5/objects/MPS_MotorDriver.axf`。

## 曲线布局

沿用一个 `type=8` 曲线窗口并启用多子图，避免手工伪造 Qt dock state：

- 子图 0：`raw16`、标定前电角度、标定后电角度。
- 子图 1：`id`、`iq`。
- 子图 2：`iu`、`iv`、`iw`。
- 子图 3：`uu`、`uv`、`uw`。

曲线描述使用业务名称和单位；变量 `name` 保持 AXF 可解析的 C 符号路径。

## 常规数值变量

数值列表至少覆盖：

- 控制状态：state、mode、fault flags、Iq/速度/位置目标。
- 编码器：raw16、corrected raw16、标定前/后电角度、机械/电气速度、valid/fresh、总采样/接受/总线错误/尖峰/stale 计数。
- 电流环：Id/Iq、窗口平均值、Iu/Iv/Iw、母线电压、Uu/Uv/Uw、PWM Ta/Tb/Tc。
- 采样重构：ADC raw、raw phase current、valid mask、重构相、三相采样 margin、sample tick、invalid/overcurrent consecutive、PI freeze、过流和不平衡计数。
- 运行诊断：ISR tick、CURRENT/SPEED/fault/disabled branch hits、编码器 alive/errors/spikes。

变量地址由完成构建后的 V1/V2 AXF 重新提取，不沿用旧文件中的历史地址。两份 AXF 必须对所有配置变量给出相同地址；若不相同，则保留相同变量集合但分别写入各自地址。

## 安全与性能边界

- 不改变电流环参数、±1.5A 命令限幅、2A 软件保护、PWM 频率或采样时刻。
- 调试相电压换算只增加三个乘加和三个整数发布，不参与控制反馈。
- LKS Scope 保持 10ms 采样周期，适用于趋势和状态监测，不宣称能捕获 16kHz 单周期波形。
- 本次不烧录、不上电、不执行电机转动测试；完成条件是主机测试、XML/AXF 检查及 V1/V2 固件构建通过。

## 验证门槛

1. `foc_inv_clarke()` 主机测试先因接口缺失而失败，再在实现后覆盖 α 轴、β 轴和三相和为零。
2. 编码器静态测试先因 `raw_elec_mrad` 缺失而失败，再验证 raw/corrected 两条计算路径。
3. LKS Scope 静态测试解析 XML，验证 11 条必需曲线、数值变量、COM9、10ms 采样及相对 AXF 路径。
4. V2、V1 分别完成现有静态测试、聚焦主机测试和 Keil clean build。
5. `fromelf` 校验所有曲线及数值变量均可从各自 AXF 解析，并核对两份地址表。
