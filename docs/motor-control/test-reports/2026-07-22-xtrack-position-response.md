# X-Track 实时 CAN 与 Node 1 位置响应联调报告

日期：2026-07-22

## 结论

通讯问题已先于位置环调参修复并通过硬件验证：X-Track 使用独立 10 ms
硬件定时器提交不可变轨迹点，发送不再依赖 LVGL 刷新；60 s 运行期间
RUNNING 状态最大 `sync_age` 为 19 ms，位置参考无冻结，协议错误、接收
溢出、Bus-off 和发送错误计数均无增长。

MotorDriver 的 `PID_POSITION_KP` 从 5.0 提高并最终保留为 7.5。它在首轮
相同工况下把 10 deg 指令的端点跨度从 7.015 deg 提高到 7.806 deg，
但最终复验为 7.212 deg，仍未达到 9.5 deg/350 ms 的响应门槛。Kp=10.0
只提高到 7.921 deg，改善不足且同样不收敛，因此按“最低有效增益”原则
回退到 7.5。CAN 和电气安全门槛在全部试验中均通过。

## 软件与硬件基线

- 协议基准：MotorDriver `2a9a004646f63013a27f1574682cc2ecae5aad40`
- X-Track 实时通讯最终提交：`c28fe9454e0093affb8d14896cea6e51fb68a403`
- MotorDriver 最终调参基础提交：`0aa594b2df16151ea6bda6afba7cf5c953be8e40`
- Node 1：J-Link `20721552`，COM9/115200
- X-Track：J-Link `20721850`
- 机械条件：电机空载、无遮挡，自动往返 +5 deg/-5 deg
- 每次异常路径和正常结束均先停 X-Track，再执行 `mc_stop`

## 三档增益对比

| Kp | 端点跨度 / 10 deg | 最大调节时间 | 最大超调 | 峰值 \|Iq_ref\| | 最大 RUNNING sync_age | CAN 错误增量 | 结论 |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 5.0 | 7.015 deg | 未进入 +/-0.5 deg | 0.000 deg | 0.218 A | 19 ms | 0/0/0/0 | 基线；跨度和调节时间失败 |
| 7.5 | 7.806 deg | 未进入 +/-0.5 deg | 0.000 deg | 0.091 A | 15 ms | 0/0/0/0 | 首轮有改善，进入 Kp=10 试验 |
| 10.0 | 7.921 deg | 未进入 +/-0.5 deg | 0.000 deg | 0.102 A | 9 ms | 0/0/0/0 | 仅比 7.5 增加 0.115 deg，拒绝 |
| 7.5 最终复验 | 7.212 deg | 未进入 +/-0.5 deg | 0.000 deg | 0.237 A | 17 ms | 0/0/0/0 | 最终固件；安全/通讯通过，响应门槛未通过 |

“CAN 错误增量”依次为 protocol error / RX overflow / bus-off / TX error。
Kp=10 和最终 Kp=7.5 日志各出现 1 个 checksum 正确但物理不可能的
单帧位置尖峰；分析器仅剔除前后位置连续、单帧跳变超过 10 deg 且下一帧
立即恢复的样本，并显式报告数量。原始日志完整保留，连续异常不会被过滤。

## 构建与回归

- X-Track MotorDemo 主机测试：通过。
- Visual Studio `LVGL.Simulator.sln` Release x64：通过。
- X-Track Keil 生产固件：0 errors / 0 warnings，Code=261264，
  RO=103848，RW=1460，ZI=382508。
- X-Track Keil commissioning 固件：0 errors / 0 warnings，Code=262304，
  RO=103848，RW=1460，ZI=382516。
- MotorDriver 30 个 Python 静态/单元脚本与严格 GCC 控制/CAN 测试：通过。
- MotorDriver 最终 Keil ARMCC5：0 errors / 0 warnings，Code=57084，
  RO=6512，RW=696，ZI=10728。
- MotorDriver 最终 HEX SHA-256：
  `DF1D3B98691D8FF9C7D8825A4747722F7EBB0AE65373BAF4FA3C20FBA0AF39C0`
- 两个用户工程文件均保持未暂存；未纳入本次提交。

## 原始证据

- [Kp=5.0 摘要](../evidence/2026-07-22-position-response-kp5.txt)
- [Kp=5.0 原始日志](../evidence/2026-07-22-position-response-kp5.raw.jsonl)
- [Kp=7.5 摘要](../evidence/2026-07-22-position-response-kp7p5.txt)
- [Kp=7.5 原始日志](../evidence/2026-07-22-position-response-kp7p5.raw.jsonl)
- [Kp=10.0 摘要](../evidence/2026-07-22-position-response-kp10.txt)
- [Kp=10.0 原始日志](../evidence/2026-07-22-position-response-kp10.raw.jsonl)
- [最终 Kp=7.5 复验原始日志](../evidence/2026-07-22-position-response-final-kp7p5.raw.jsonl)

最终复验原始日志 SHA-256：
`7EBEB5948F17CA0DBDAB094EC9A75366EB372EC2919EDD7516BB6BC4F18E63F1`。
采集结束记录为 `DISABLED`、fault=0、EN=0、CCR1/2/3=2812/2812/2812。

## 后续调参方向

本轮单变量试验说明继续小幅增加位置 Kp 的收益很低，同时电流限幅尚有
充足余量。下一轮不应盲目继续加 Kp；应先把速度环阶跃响应与静摩擦补偿
分别量化，再一次只调整一个位置专用参数。这样才能区分“位置环太软”是
速度内环带宽不足、摩擦死区，还是演示轨迹端点保持时间不足造成的。
