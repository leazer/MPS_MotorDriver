# 单电机位置与速度前馈闭环设计

## 目标

在已经通过台架验证的电流环和速度环之上，实现一个可供五连杆上位机复用的单电机执行器：接收机械位置点、机械速度前馈、序号和流式命令租约，在电机本地完成位置外环、速度内环和电流内环；首先通过 COM9 完成控制性能与安全行为验收，真实 CAN 和双节点同步另立任务。

本阶段的“单电机可用”不是静态位置命令能够转到某处，而是以下能力同时成立：

- 位置和速度前馈使用机械关节坐标，正方向与现有编码器归一化方向一致。
- 100 Hz 离散点之间由电机本地外推参考，1 kHz 位置外环连续生成电气速度目标。
- 正常结束保持末点；流式命令超时冻结参考并清零速度前馈；显式停止或致命故障才关闭驱动。
- 位置目标更新不重启速度环或清空内环状态。
- COM9 自动台架能够测量静态误差、动态跟踪误差、超调、速度、电流、采样质量、超时和最终安全态。

## 范围与非目标

### 本阶段包含

1. 方向归一化、校准后的连续机械位置接口。
2. 运行时关节坐标零点，开机后由操作员在已知姿态捕获。
3. 位置 P 环与机械速度前馈，输出电气速度目标给现有速度环。
4. 位置模式 ISR 控制链和启动、更新、停止接口。
5. COM9 静态目标与 100 Hz 流式点命令。
6. 紧凑遥测、主机自动测试、固件构建、烧录和自由轴台架调参。

### 本阶段不包含

- AT32M412 CAN1 驱动、CAN 协议解析或状态上报。
- X-Track `CanMotorTransport`。
- 双电机节点地址、预装点加广播 SYNC 或机构联动。
- 关节零点 Flash 持久化或机械限位/回零开关。
- 齿槽转矩图、扭矩脉动补偿或位置积分环。

## 已知基础与量程匹配

- FOC 频率 16 kHz，速度 PI 内部以 1 kHz 更新。
- 编码器以 4 kHz 采样，速度估计使用 4 ms 窗口。
- 当前速度环参数为 `Kp=0.01`、制动 `Kp=0.04`、`Ki=0.01`、摩擦前馈 20 mA、Iq 限制 0.5 A、速度斜坡 50 electrical rad/s^2。
- X-Track 五连杆临时几何限制为 60 deg/s 和 300 deg/s^2。7 极对换算后约为 70 electrical rpm 和 36.7 electrical rad/s^2，处于现有速度环已验证范围内，并低于速度斜坡限制。
- 默认 MPS Logo 轨迹周期为 10 ms，因此单电机 setpoint 接口固定按 100 Hz 设计，但控制器不依赖 COM9 精确定时。

## 控制坐标

### 传感器连续位置

现有 `raw16`、`corrected_raw16`、`raw_unwrapped` 和 `corrected_unwrapped` 保持传感器诊断语义。新增控制机械位置：

```text
control_position_mdeg =
    (corrected_unwrapped - electrical_zero_raw)
    * MOTOR_ENCODER_DIRECTION
    * 360000 / 65536
```

计算使用 64 位中间值。它是连续、多圈、方向归一化并应用非线性校准的位置，但零点仍是 FOC 电角度标定零点，不等于机构关节零点。

### 运行时关节零点

位置环保存：

```text
joint_offset_mdeg = known_joint_position_mdeg - control_position_mdeg
joint_position_mdeg = control_position_mdeg + joint_offset_mdeg
```

`mc_pos_zero [known_mdeg]` 只能在电机停止时执行，默认把当前位置定义为 0 mdeg。未捕获关节零点时禁止进入 POSITION 模式。该零点本阶段不写 Flash，重启后必须重新捕获。

## Setpoint 契约

统一输入类型：

```c
typedef struct {
    int32_t position_mdeg;
    int32_t velocity_mdeg_s;
    uint16_t sequence;
    uint16_t lease_ms;
} position_setpoint_t;
```

- `position_mdeg`：逻辑关节机械位置。
- `velocity_mdeg_s`：逻辑关节机械速度前馈。
- `sequence`：16 位单调序号，按半范围规则允许回绕；倒序点拒绝。同序号且位置、速度、租约完全相同的点按幂等重发接受，内容不同则拒绝。
- `lease_ms=0`：静态保持命令，不因缺少新点超时。
- `lease_ms=100`：100 Hz 流式命令；100 ms 未收到更新时进入超时保持。

线程或 CAN 上下文发布 setpoint，16 kHz ISR 消费。发布使用奇偶 generation 防止位置、速度和序号被读成跨帧组合；ISR 仅消费完整偶数 generation。

## 参考外推与超时

位置环以 1 kHz 更新。收到新点后：

```text
reference_position = command_position
                   + velocity_ff * min(age, 20 ms)

speed_reference_mech = velocity_ff
                     + Kp_position * position_error

speed_reference_elec = pole_pairs * speed_reference_mech
```

位置和速度在换算前使用 mdeg、mdeg/s，最终转换成 rad/s。外推最多 20 ms，防止调度抖动造成 10 ms 台阶，同时避免长期丢包时参考继续漂移。

当流式 setpoint 超过 `lease_ms`：

1. 把当前已经外推的参考位置冻结。
2. 把速度前馈置零。
3. 保持 POSITION 模式，用位置 P 环把电机留在冻结位置。
4. 置 `stream_timeout` 诊断位；新序号点到达后允许恢复。

这与显式 `mc_stop` 不同。`mc_stop` 清位置/速度/电流环、关闭 PWM 中断和 MP6540H EN，进入 DISABLED。

## 位置外环

初始参数：

```text
Kp_position = 5.0 s^-1
max_speed_electrical = 200 rpm
max_velocity_feedforward = 60 deg/s mechanical
max_position_error = 30 deg
```

自由轴实测增加仅在 POSITION 模式生效的库仑摩擦前馈：静态误差超过 0.2 deg 时为 80 mA，流式速度非零时为 40 mA。速度环本身的 20 mA 摩擦前馈和独立 SPEED 模式参数不变；位置级联总 `Iq_ref` 仍限制在 ±0.5 A。该分档用于克服静摩擦，同时避免运动中 80 mA 造成约 20% 的轨迹幅值超前。

控制律不加入积分和微分。原因是绘图优先平滑轨迹，速度前馈承担主要动态跟踪，P 环只消除位置误差；位置积分会在限流或机构卡滞时积累，当前阶段收益小、风险高。

超过位置误差、速度前馈或目标位置范围时拒绝启动/更新，而不是静默使用危险值。位置误差运行门限触发位置模式停止并报告故障；普通流式超时只进入保持。

## ISR 级联

POSITION 分支复用 SPEED 分支的编码器角度、电流采样、Clarke/Park、电流 PI、IPark 和 SVPWM。有效电流帧到达时：

1. 读取方向归一化的连续机械位置。
2. 每 1 ms 更新位置环，得到电气速度目标。
3. 调用 `speed_loop_set_target_rad_s()`，再执行现有 `speed_loop_run()`。
4. 把速度环输出与 POSITION 专用摩擦前馈相加、钳位到 ±0.5 A，再作为 Iq 目标执行现有电流环。

POSITION 模式启动只重置一次位置、速度和电流环；后续 setpoint 更新不得重置任何内环。

## COM9 命令

```text
mc_pos_zero [known_mdeg]
mc_pos <position_mdeg>
mc_pos_stream <sequence> <position_mdeg> <velocity_mdeg_s>
mp <sequence> <position_mdeg> <velocity_mdeg_s>
mc_pos_status
mc_stop
```

- `mc_pos` 用 `lease_ms=0` 启动或更新静态保持目标。
- `mc_pos_stream` 用 `lease_ms=100` 启动或更新流式目标；`mp` 是相同命令的短别名，用于降低 COM9 调试链路的字符扰动概率。
- 第一个位置命令从 DISABLED 启动 POSITION；运行中命令只提交新点。
- 从 SPEED/CURRENT/OPEN_LOOP 等其他活动模式切入时拒绝，要求先 `mc_stop`。

紧凑遥测包含 activity、目标、前馈、参考、实测、误差、电气速度目标/实测、Iq、age、timeout、sequence、fault 和 XOR 校验。主机只接受完整且校验正确的行，避免 COM9 静默截断被误判为有效数据。

## 测试策略

### 主机 RED/GREEN

1. 连续机械位置覆盖方向、校准、跨 0/65535 和多圈。
2. 位置环覆盖零点未设置、静态目标、速度前馈、1 kHz 外推、20 ms 外推上限、100 ms 超时冻结、序号重复/倒序/回绕、速度限幅和重置。
3. 静态检查锁定 POSITION ISR 必须调用位置环和既有速度/电流链，位置更新不得重启模式。
4. Python 台架解析器覆盖完整行、校验错误、截断行、超时和异常路径必调 `mc_stop`。
5. 全部现有编码器、电流环、速度环和采样重构测试回归。

### 自由轴台架

所有上电测试使用 0.5 A 速度环限制，先小幅、再动态；每段后执行 `mc_stop`。

1. 捕获当前位置为 0，运行 +5 deg、-5 deg 静态阶跃。
2. 运行 +10/-10 deg 反转和末点保持。
3. COM9 文本链路按 50 Hz 名义周期发送幅值 10 deg、峰值速度不超过 30 deg/s 的正弦位置与解析速度前馈；轨迹按实际点间隔推进，单次异常间隔钳到 40 ms，并单独报告有效点率与重试次数。生产 CAN 接口仍按 100 Hz 设计。
4. 在动态段停止发送超过 100 ms，验证参考冻结、前馈归零、没有继续漂移。
5. 重发更新序号，验证从保持平滑恢复。
6. 最终验证 DISABLED、EN=LOW、三相 PWM=2812、CCR4=5264、fault=0。

## 单电机验收门禁

- 静态 ±5 deg 目标在稳定后绝对误差不超过 0.5 deg，无持续来回抖动。
- ±10 deg 反转无失控，超调不超过目标幅值的 20%，Iq 绝对值不超过 0.5 A。
- COM9 资格测试的 10 deg/30 deg/s 正弦轨迹 P95 跟踪误差不超过 2 deg，末点速度前馈为零后能保持；100 Hz CAN 时序留到 CAN 集成阶段实测。
- 流式超时后 30 ms 内速度前馈清零且参考不再继续外推；电机保持而非继续运动。
- 整段测试 fault=0，采样 invalid/freeze 不持续增长，串口帧全部通过校验。
- 主机测试、静态检查和 ARMCC5 固件构建为 0 error、0 warning。

如果由于机械惯量或编码器低速纹波使动态门禁无法同时满足，先只调 `Kp_position`、位置速度上限和测试轨迹速度；不得在本阶段加入位置积分或齿槽补偿来掩盖级联问题。

## 后续 CAN 接口边界

本阶段的 `position_setpoint_t` 直接对应后续单节点 8 字节轨迹帧：

```text
position_mdeg      int32 LE
velocity_10mdeg_s  int16 LE
sequence           uint16 LE
```

CAN 接收层把速度放大 10 倍、设置 `lease_ms=100` 后调用同一提交接口。双电机任务再增加 pending setpoint、广播 SYNC、节点状态反馈和 X-Track Transport，不修改本阶段位置控制律。

对 X-Track `five-bar-motor-demo-offline` 的核对结论：`TrajectoryPoint` 已同时包含两关节位置与速度，规划周期固定为 10 ms，`IMotorTransport::Submit()` 的抽象也可直接承载该点；但 `CanMotorTransportStub` 仍主动阻止硬件运动。X-Track 侧已有 `HardwareCAN::write()` 基础，电机侧只有 CAN 引脚和 `can_protocol` stub，双方仍需共同冻结 CAN ID、节点号、状态/心跳和双电机同步策略后实现真实 Transport。控制器与轨迹数据结构之间不存在算法或量程阻塞。
