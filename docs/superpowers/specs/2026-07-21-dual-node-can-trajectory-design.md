# 双节点 CAN 轨迹与持久关节零点设计

## 目标

在已经通过自由轴验证的单电机位置、速度前馈、速度环和电流环之上，为五连杆演示建立可复用的双节点执行器接口：X-Track 通过一条 1 Mbps Classic CAN 总线连接两块相同的 MPS MotorDriver，每块板驱动一个电机；X-Track 以 100 Hz 分别预装两个关节的“位置 + 速度前馈”点，再用广播 `SYNC` 使两节点在各自下一个 1 kHz 控制节拍应用同一序号。

本设计首先把一块板做到“单电机 CAN 可用”，随后复用同一固件配置第二节点，最后才进入双电机空载和五连杆联动。目标轨迹是流畅绘制 MPS 字样；本阶段优先保证位置轨迹、同步和失联安全，不引入齿槽转矩图或扭矩脉动补偿。

## 范围与非目标

### 本阶段包含

1. AT32M412 CAN1 的 1 Mbps 驱动、过滤器、接收队列和非阻塞发送。
2. 固定 8 字节 Classic CAN 协议、节点寻址、广播控制、反馈与健康状态。
3. 双节点预装点加广播同步，以及序号、会话和超时规则。
4. 100 Hz 离散轨迹点到本地 1 kHz 位置参考的重构。
5. 节点 ID、关节机械零点、方向和允许范围的掉电保存。
6. X-Track 真实 `CanMotorTransport`、双节点反馈合并和联动停止。
7. 单节点、第二节点、双节点空载及最终机构低速验收。

### 本阶段不包含

- 单块驱动板同时驱动两个电机。
- CAN FD、扩展帧或总线时间同步协议。
- 每次上电自动撞限位回零；限位开关仅预留为以后自动标定或零点复核手段。
- 齿槽转矩表、位置积分、重复轨迹学习或其他扭矩脉动补偿。
- 修改 X-Track 已有五连杆轨迹规划算法；只实现其硬件传输边界。

## 已知基础

- 两个电机分别由两块相同驱动板控制，固件相同，仅持久配置的节点 ID 和关节参数不同。
- MA600A 直接连接电机输出轴，是单圈绝对角度传感器，不存在减速器输入输出多圈映射问题。
- 单电机位置接口已经接收机械位置、机械速度前馈和 16 位序号，并在 1 kHz 位置环内对 100 Hz 点进行参考外推。
- X-Track 的 `TrajectoryPoint` 已包含 `q1/q2` 位置、`q1/q2` 速度、序号和 10 ms 时间戳；`IMotorTransport::Submit()` 可以承载完整双关节点。
- X-Track 已有 Classic CAN `HardwareCAN` 基础，但当前 `CanMotorTransportStub` 会阻止真实硬件运动；MotorDriver 的 `can_at32m412` 和 `can_protocol` 仍是占位实现。

## 总体架构

```text
X-Track trajectory/controller (100 Hz)
            |
            v
CanMotorTransport
  |- rad <-> mdeg / mdeg/s
  |- Node 1 preload
  |- Node 2 preload
  |- broadcast SYNC / STOP
  `- merge same-sequence feedback
            |
         CAN 1 Mbps
       /             \
      v               v
MotorDriver Node 1   MotorDriver Node 2
  can_at32m412         can_at32m412
  can_protocol         can_protocol
  can_motion_service   can_motion_service
       |                    |
 position -> speed -> current loops (local 1 kHz / 16 kHz)
```

MotorDriver 分层职责如下：

- `can_at32m412`：GPIO、时钟、1 Mbps 位时序、标准帧过滤、RX 中断环形队列、非阻塞 TX 和硬件错误统计。它不解释电机业务字段。
- `can_protocol`：纯函数形式的 ID、长度、字节序、CRC8、定点单位和饱和检查编解码，不访问硬件或控制器。
- `can_motion_service`：节点状态、pending setpoint、`ARM/SYNC/STOP`、会话、序号、超时、状态快照，以及对既有位置 setpoint API 的唯一调用边界。
- 既有位置、速度和电流控制律保持不变；CAN 只替换命令来源并把流租约收紧到本设计规定的时间。

CAN RX 中断只把已通过硬件帧校验的标准帧复制到定长 SPSC 队列。1 kHz 控制服务每次有界地处理命令队列，先处理预装点与同步事件，再运行位置环，因此 `SYNC` 后的应用时刻不依赖串口或主循环耗时。状态发送读取控制侧快照，不在控制中断里等待邮箱。

## 物理总线与时序预算

- Classic CAN，标准 11 位 ID，1 Mbps。
- 线束短，主干两端各放置 120 Ω 终端；避免星形长支线。
- X-Track 每 10 ms 发送两帧节点轨迹和一帧广播 `SYNC`。
- 两节点各以 100 Hz 返回快速反馈，并以 20 Hz 返回健康状态；故障状态立即额外上报一次。
- 约 500 至 550 帧/秒。按每个 8 字节标准帧含最坏填充约 130 bit 保守估算，正常负载低于 8%，为仲裁重发、调试和未来状态帧保留足够余量。
- 三个轨迹命令帧的线速发送时间约 0.4 ms；两个节点在收到同一广播后于各自下一个 1 kHz 控制节拍应用，设计同步偏差上界约为一个控制节拍，即 1 ms。

## CAN 标识符和负载

所有多字节整数均为小端。未列出的 DLC 一律拒绝；控制帧只在节点已配置且状态允许时解释。

首版 `protocol_version` 固定为 `1`。广播帧 `flags` 的所有位首版均保留并必须为零；收到非零保留位时拒绝该帧。`crc8` 使用 CRC-8/ATM（poly `0x07`、init `0x00`、refin/refout false、xorout `0x00`）覆盖 byte 0 至 byte 6。

### 节点轨迹预装：`0x100 + node_id`

Node 1 为 `0x101`，Node 2 为 `0x102`，DLC 固定为 8：

```text
byte 0..3  position_mdeg       int32
byte 4..5  velocity_10mdeg_s   int16
byte 6..7  sequence            uint16
```

`velocity_10mdeg_s` 每计数代表 10 mdeg/s，即 0.01 deg/s；范围为约 ±327.67 deg/s，覆盖当前 60 deg/s 规划限制。节点只把合法点保存为 pending，不立即改变控制目标。位置越界、速度越界、旧序号或同序号不同内容均拒绝；完全相同的重复帧按幂等重发接受。

### 广播控制：`0x080`

DLC 固定为 8，广播 ID 的仲裁优先级高于节点轨迹与反馈：

```text
byte 0     opcode
byte 1     protocol_version
byte 2..3  sequence            uint16
byte 4..5  session             uint16
byte 6     flags
byte 7     crc8
```

首版 opcode：

- `0x01 ARM`：建立新 session、清除 pending 和旧序号窗口，进入 `ARMED`；ARM 中的 sequence 是该 session 预期的第一个轨迹序号。
- `0x02 SYNC`：仅当 session 正确且 pending sequence 完全匹配时应用该点。
- `0x03 STOP`：任何状态均立即执行清 pending、清速度前馈和关闭输出；活动状态回到 `READY`，但不得清除锁存故障或修复无效配置。
- `0x04 CLEAR_FAULT`：仅在硬件故障条件已经消失且配置仍有效时清除锁存故障，回到 `READY`，不得直接恢复运动。
- `0x05 DISCOVER`：请求已配置节点立即发送健康状态，不改变控制状态。

节点更换 session 时必须清空 pending。在该 session 首点应用前只接受 ARM 指定的第一个 sequence；之后按 16 位半范围规则只接受更新序号。X-Track 每次启动控制生成新 session 和新的初始 sequence，并在 ARM 前清空自身待发送队列，降低旧排队帧与新 session 序号偶合的可能性。`STOP` 不依赖当前 session，确保失控时不会因会话不同而被拒绝。

### 快速反馈：`0x180 + node_id`

Node 1 为 `0x181`，Node 2 为 `0x182`，DLC 固定为 8，正常 100 Hz：

```text
byte 0..3  actual_position_mdeg       int32
byte 4..5  actual_velocity_10mdeg_s   int16
byte 6..7  applied_sequence           uint16
```

`applied_sequence` 是位置环实际消费的最后序号，而不是最近收到的 pending 序号。X-Track 只有在两节点反馈的 applied sequence 相同且等于预期序号时，才把组合反馈交给上层并推进“已应用”进度。

### 健康状态：`0x280 + node_id`

Node 1 为 `0x281`，Node 2 为 `0x282`，DLC 固定为 8，正常 20 Hz，状态变化或故障时立即发送：

```text
byte 0     protocol_version
byte 1     node_state
byte 2..3  fault_bits           uint16
byte 4..5  session              uint16
byte 6..7  vbus_10mV            uint16
```

健康帧用于确认协议版本、状态、当前 session、母线电压及 `FAULT_CAN_TIMEOUT` 等锁存故障。快速反馈负责实时闭环进度，健康帧不承担 100 Hz 位置数据。

`node_state` 首版编码固定为：`0 UNCONFIGURED`、`1 READY`、`2 ARMED`、`3 RUNNING`、`4 HOLD`、`5 FAULT`。

## 双节点提交与同步

每个 10 ms 周期由 X-Track 执行：

1. 将 `q1` 和 `dq1` 换算、检查并发送给 Node 1。
2. 将 `q2` 和 `dq2` 换算、检查并发送给 Node 2。
3. 发送带相同 sequence 和当前 session 的广播 `SYNC`。
4. 接收两节点反馈；只有两者确认同一 applied sequence 才更新组合反馈。

节点收到自己的轨迹帧后保存 pending。收到匹配 `SYNC` 后，1 kHz 控制服务原子地把 pending 发布给既有 `position_setpoint_t`，设置 50 ms CAN 流租约并记录 applied sequence。广播早于 pending、session 不符或 sequence 不符时不得部分应用。

未被匹配 `SYNC` 的 pending 在 30 ms 后丢弃。重复 `SYNC` 对已经应用的同一序号是幂等操作；旧序号按 16 位半范围规则拒绝，从而允许 `65535 -> 0` 正常回绕。

两个节点的 1 kHz 时钟不做相位锁定，因此设计保证的是接收同一 `SYNC` 后各自在下一控制节拍应用，预期偏差不超过约 1 ms，而不是微秒级同时更新。该精度远高于五连杆 100 Hz 轨迹周期的需要。

## 100 Hz 点间重构

CAN 点继续使用已经验证的单电机位置接口。每个点同时包含位置和速度前馈，本地位置环在 1 kHz 使用：

```text
reference_position = command_position
                   + velocity_ff * min(point_age, extrapolation_limit)

speed_reference = velocity_ff
                + Kp_position * position_error
```

因此 10 ms 内不是保持阶梯位置，而是按速度前馈连续推进参考。首版保留已验证的线性外推和 20 ms 外推上限；不在 CAN 层另做插值。以后如轨迹曲率需要，可在不改变帧格式的情况下升级为使用相邻点的 Hermite 重构，但不作为 MPS 字样演示的前置条件。

## 节点配置和持久关节零点

每块板通过 COM 在电机输出关闭时配置一次。持久记录包含：

```text
format_version
generation
node_id
zero_corrected_raw16
known_joint_position_mdeg
joint_direction
min_joint_position_mdeg
max_joint_position_mdeg
crc32
```

- `node_id` 首版只允许 1 或 2。
- `zero_corrected_raw16` 是标定姿态下经过编码器非线性校准的 MA600A 单圈读数。
- `known_joint_position_mdeg` 是该姿态在五连杆关节坐标中的角度。
- `joint_direction` 只能为 `+1` 或 `-1`。
- 允许范围必须有效且小于一整圈，使上电角度解唯一。
- `format_version` 和 CRC 防止旧格式或损坏数据被当成有效零点。

Flash 使用两个独立可擦除记录区域和递增 generation：只在电机停止且 PWM/驱动使能均关闭时擦写非当前有效区域，写完回读并校验后才把它视为最新记录。两个逻辑槽不得放在擦除时会同时丢失的同一擦除页内。具体页地址必须在实施时依据链接脚本和芯片容量确定，不在设计阶段硬编码。这样可避免标定写入时掉电直接破坏最后一个有效记录。

建议的串口能力包括配置当前姿态、显示配置和擦除配置；具体命令名服从现有 CLI 风格。配置当前姿态时，由固件捕获有效的 `corrected_raw16`，不允许操作者手工录入传感器原始值。

上电恢复流程：

1. 选择 generation 最新且版本、字段和 CRC 均有效的记录。
2. 等待编码器校准完成并取得稳定的 `corrected_raw16`。
3. 计算相对保存零点的模 65536 角度差，并结合方向和已知逻辑角度生成相差整圈的候选值。
4. 在 `[min_joint_position, max_joint_position]` 内选择唯一候选值。
5. 用该逻辑角度初始化位置环关节坐标原点，进入 `READY`。

无候选或存在多个候选均判定零点无效，保持输出关闭并进入 `UNCONFIGURED`。由于 MA600A 直接连输出轴，且机构合法范围小于 360°，正常情况下可以一次标定后跨断电恢复。机械限位开关以后可用于首次自动捕获或周期复核，但首版运行不依赖限位 GPIO。

## 节点状态机

节点状态为：

- `UNCONFIGURED`：节点或零点记录无效，只允许串口配置与本地查询，禁止运动；由于没有可信 node ID，不在 CAN 上响应发现。
- `READY`：配置、编码器和驱动诊断正常，输出关闭，等待 ARM。
- `ARMED`：已建立 session、pending 为空、输出仍关闭，等待首个合法同步点。
- `RUNNING`：已应用同步轨迹点，位置模式有效。
- `HOLD`：有效 `SYNC` 中断达到 50 ms，冻结当前已外推位置并把速度前馈清零，位置环继续保持。
- `FAULT`：CAN 超时达到 500 ms、编码器异常、关节越界或已有致命驱动故障；关闭输出并锁存故障。

首版状态值使用上一节健康帧中冻结的 `0..5` 编码。已有 `FAULT_CAN_TIMEOUT` 加入致命故障掩码；另新增致命 `FAULT_CAN_BUS` 表示 bus-off、运行中 RX 队列溢出或不可恢复的 TX 故障。无效关节配置由 `UNCONFIGURED` 表达，不复用当前用于电流采样校准告警的 `FAULT_CAL_INVALID`。

关键转换：

```text
valid config:  UNCONFIGURED -> READY
ARM:           READY -> ARMED
first SYNC:    ARMED -> RUNNING
50 ms silence: RUNNING -> HOLD
valid SYNC:    HOLD -> RUNNING
500 ms silence:HOLD -> FAULT_CAN_TIMEOUT
STOP:          ARMED/RUNNING/HOLD -> READY
CLEAR_FAULT:   FAULT -> READY (only when safe)
```

首个同步目标若与恢复后的实际位置相差超过既有位置安全门限，节点拒绝启动并上报故障，避免 ARM 后突然跳到远处。进入 `FAULT` 后即使通信恢复也不得自动重新使能，必须清故障、重新 ARM 并接收首个同步点。

## X-Track 传输行为

真实 `CanMotorTransport` 实现现有 `IMotorTransport`，不改变轨迹控制器接口：

- `Submit(point)` 对两个关节执行 rad/rad/s 到 mdeg/mdeg/s 的有限值、范围和饱和检查，再依次发送两个预装帧和一个 `SYNC`。
- 发送任一帧失败时，本周期不视为成功；立即进入停止流程，不允许只让一个节点继续。
- 反馈必须来自 Node 1 和 Node 2，协议版本、session、状态均正确，且 applied sequence 一致，才合成为一个 `MotorFeedback`。
- 任一节点快速反馈超过 30 ms 未更新、两节点序号不一致持续 30 ms、节点报告故障、session 不符或零点无效时，X-Track 广播 `STOP` 并阻止轨迹继续。
- `STOP` 至少重复发送三次，但 API 调用保持非阻塞；节点侧仍独立执行 50/500 ms 看门狗，不能把安全完全委托给 X-Track。
- 启动流程先 `DISCOVER` 并确认 Node 1 和 Node 2 均有版本一致的 `READY` 响应，再建立新 session、广播 ARM，最后开始轨迹。

仅凭标准 CAN ID 和当前 8 字节健康帧，主机不能可靠证明总线上不存在第二个相同 node ID 的设备。重复 ID 必须在装机流程中避免：每次只连接一块新板完成 COM 配置和独立发现，记录其 Node 1/2 身份后才同时接入；双板接入后的仲裁错误、bus-off 或状态异常会阻止 ARM，但不宣称这是完整的重复身份检测。如以后需要热插拔自动识别，应另增基于芯片唯一 ID 的发现扩展帧。

## 错误处理与诊断

- CAN 硬件 error-passive 进入通信诊断；bus-off、运行中 RX 溢出和 TX 长期失败立即安全停机并锁存 `FAULT_CAN_BUS`。
- 协议 DLC、版本、节点号、CRC8、单位范围或状态不合法时丢帧并增加对应计数，不修改当前目标。
- RX 队列采用固定容量且禁止动态分配；溢出时不覆盖未处理命令，运行中按通信故障处理。
- TX 状态队列不得阻塞控制。普通反馈可以丢弃旧快照保留最新值；STOP 和故障状态的优先级高于周期遥测。
- 串口状态命令应能观察 node ID、状态、session、pending/applied sequence、最后同步年龄、RX/TX/协议错误计数、零点有效性和故障位。

## 测试策略

### MotorDriver 主机测试

1. 协议编解码：所有帧的 ID、DLC、小端字段、正负边界、饱和、CRC8 和非法版本。
2. 序号：首次、递增、重复幂等、同序号不同内容、倒序和 `65535 -> 0` 回绕。
3. 同步服务：pending 后 SYNC、SYNC 早到、错误 session、错误 sequence、重复 SYNC、30 ms pending 过期。
4. 安全时序：50 ms 进入 HOLD、冻结参考与前馈归零、500 ms 进入 `FAULT_CAN_TIMEOUT`、STOP 优先、故障后不自动恢复。
5. 持久配置：版本、CRC、双槽 generation、写后回读、原始角度跨零、方向、唯一范围候选、无解和多解拒绝。
6. 静态检查：RX ISR 不阻塞，控制节拍处理量有界，TX 不在控制 ISR 等待邮箱，CAN setpoint 复用既有位置 API。
7. 全部现有编码器、电流环、速度环、位置环和采样测试回归，ARMCC5 构建保持 0 error、0 warning。

### X-Track 测试

1. 使用模拟 `HardwareCAN` 验证每个点恰好产生 Node 1、Node 2 和 SYNC 三帧，字段换算和发送顺序正确。
2. 两节点相同序号才合并反馈；缺节点、错 session、错版本、故障、丢反馈和序号分歧均触发 STOP。
3. ARM、发现、停止重发、序号回绕和 CAN 写失败路径不允许控制器误判为已经应用。
4. 既有离线轨迹规划和 10 ms 调度测试继续通过。

### 硬件分阶段验收

#### 阶段 A：单节点 CAN

- 只接当前已验证电机作为 Node 1，1 Mbps、100 Hz 连续运行至少 10 分钟，无 bus-off、RX 溢出或不可解释丢序。
- 人工中断轨迹，验证 50 ms HOLD、500 ms 关闭输出和锁存 `FAULT_CAN_TIMEOUT`。
- 断电重启后节点 ID 和关节零点正确恢复；配置损坏时保持输出关闭。
- 10°幅值、峰值 30 deg/s 正弦轨迹的 P95 位置误差目标不高于约 1.5°，峰值 Iq 不超过现有 0.5 A 限制，效果不得明显差于已验证的本地/串口输入。
- 再逐步验证峰值 60 deg/s，记录误差与电流，不为通过测试而放宽安全限流。

#### 阶段 B：第二节点

- 同一固件烧录第二块板，通过 COM 配置 Node 2 和其关节零点。
- 第二电机先独立完成编码器、方向、静态位置、正反转和 CAN 轨迹验收。
- 两块板禁止使用相同 node ID；分别单板配置和发现确认身份后才能同时接入。双板接入后缺失任一 ID、出现仲裁/总线错误或状态异常时不得 ARM。

#### 阶段 C：双节点空载与机构

- 双节点空载运行相同和相反方向轨迹，反馈序号持续一致，无单边运行。
- 用两节点调试 GPIO 在实际应用新 sequence 时翻转，示波器验证同步偏差不超过约 1 ms。
- 拔除任一节点 CAN 或制造任一节点故障，X-Track 对两节点广播 STOP，另一节点不得继续轨迹。
- 安装五连杆后先使用小角度、低速度和现有限流验证方向、零点与工作空间，再运行 MPS 字样轨迹。

## 实施顺序与仓库边界

1. 在 MotorDriver 完成纯协议、持久配置、motion service 和 CAN 硬件驱动的测试驱动实现。
2. 将单块板烧录为 Node 1，用脚本或 X-Track 最小发送端完成单电机 CAN 台架资格测试。
3. 在 X-Track `five-bar-motor-demo-offline` 实现真实 `CanMotorTransport` 及其模拟 CAN 测试。
4. 配置并验证 Node 2，再做双节点同步与故障联动。
5. 最后安装机构和运行 MPS 轨迹；若此时仍存在可感知周期性误差，再基于误差频谱判断是否需要扭矩脉动补偿。

MotorDriver 与 X-Track 的协议常量必须各自有字节级契约测试，并使用本文件作为唯一语义来源。首轮实现不得一边联调一边临时改变单位、ID 或超时；任何协议变更先更新本设计和两侧测试。

## 完成门禁

- 1 Mbps/100 Hz 单节点连续通信、断流安全、持久零点和轨迹性能全部通过。
- 两块板使用相同固件和不同持久 node ID，第二电机独立资格测试通过。
- X-Track 能发现、ARM、同步提交、合并反馈并在任一异常时联停两节点。
- 双节点 applied sequence 一致，实测应用偏差约不超过 1 ms。
- 五连杆低速轨迹无错误方向、零点跳变、单边继续或通信失控。
- 两仓库相关主机测试通过，MotorDriver ARMCC5 构建 0 error、0 warning，最终停止态为驱动关闭且无非预期故障。
