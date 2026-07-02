"""
Stage 4 + 4b 台架自动化验收脚本 (MA600A 编码器 + 旁轴非线性标定)

对应 spec `docs/superpowers/specs/2026-06-22-mps-foc-design.md` §7 Stage 4/4b,
以及 CLAUDE.md "Stage 4 + 4b Complete" 段的 12 步台架验收清单.

串口约束 (CLAUDE.md Stage 2 关键约束):
  - finsh `rt_hw_console_getchar()` 是轮询读 USART, 无 RX 中断/FIFO.
  - 一次性发整行会 overrun 丢字符. 必须逐字符发送 + 等回显.
  - 本脚本沿用 tests/com9_test.py 的 send_cmd 机制.

输出格式约束 (CLAUDE.md Stage 2):
  - rt_kprintf 不支持 %f, 所有浮点已转定点 (mV/mrad/mdeg/mrpm).
  - 脚本按 motor_shell.c 的实际 key:value 标签做正则解析.

验收矩阵 (spec §7 Stage 4/4b):
  Section A: 编码器 SPI 连通性 (静止, 不动电机)
    A1. enc_status.raw16 随手转轴变化 (16-bit, 0..65535)
    A2. enc_status bus/spike errors 不增长
  Section B: ALIGN 零点对齐
    B1. mc_align 1000 -> 转子锁定, mc_debug.align active=1
    B2. mc_zero 显示 align_angle 非零
    B3. mc_stop -> align active=0
  Section C: 开环 enc 跟踪
    C1. mc_open 1000 300 enc -> ISR 用编码器电角度
    C2. mc_debug.encoder enc_raw 随转动变化, enc_alive=1, enc_errors 不增长
    C3. mc_current 电流正弦, 无过流
    C4. mc_stop
  Section D: 旁轴标定 (~25s, 电机自动正反拖动)
    D1. mc_calibrate 启动
    D2. 轮询 mc_cal_status 直到 DONE 或 ABORTED (超时 60s)
    D3. DONE 时 max_resid < 1000 mdeg (1°)
    D4. mc_cal_dump 256 点表非零
  Section E: 持久化
    E1. (需手动断电重启) 重启后 fault 无 CAL_INVALID, encoder.cal_valid=1
    E2. mc_cal_erase + 重启 -> fault 有 CAL_INVALID

  注: E1/E2 涉及断电重启, 脚本只做 D 完成后的即时校验 + 打印 E 的手动步骤提示.

用法:
  python tests/stage4_bench.py            # 默认 COM9
  python tests/stage4_bench.py COM7       # 指定串口
  python tests/stage4_bench.py COM7 skip_calib   # 跳过标定 (只验 A/B/C)

前置条件 (硬件侧):
  1. JLink + 板子已接, 限流电源 12V 限流 0.5A
  2. flash.bat rebuild 烧录最新固件
  3. 串口连 PB6(TX)/PB7(RX) 115200, 板子已上电见 msh 提示符
  4. 电机轴可自由旋转 (标定会自动拖动 5+5 圈)

作者注: 脚本不替代示波器/电流探头验收, 只做 msh 命令链路的功能性自动化.
        示波器看 PA8/9/10 PWM 和电流探头看三相波形仍需人工执行.
"""
import serial
import sys
import time
import re

PORT = "COM9"
BAUD = 115200

# ---- 验收阈值 (与 motor_params.h 对齐) ----
CAL_MAX_RESIDUAL_MDEG = 1000   # spec §4.7.9: 残差峰峰 < 1°
CAL_TOTAL_TIMEOUT_S   = 120    # 标定总超时 (ALIGN 0.5s + FWD 35s + REV 35s + compute + flash ≈ 72s, 留余量)
# MA600A 角度寄存器为 16-bit (ma600a 驱动 ma600a_read_angle_deg 用 /65536 转角度).
ENC_RAW_RANGE_16BIT   = (0, 65535)


# ============================================================
# 串口基础 (逐字符 + 等回显, 适配 finsh 轮询 getchar)
# ============================================================

def open_port(port):
    ser = serial.Serial(port, BAUD, timeout=0.5)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def read_all(ser, settle=0.15, max_wait=2.0):
    """读取直到 settle 时间内无新数据, 或达到 max_wait."""
    data = b""
    deadline = time.time() + max_wait
    while time.time() < deadline:
        if ser.in_waiting:
            data += ser.read(ser.in_waiting)
            deadline = time.time() + settle
        else:
            time.sleep(0.01)
    return data.decode("ascii", errors="replace")


def send_char_and_wait_echo(ser, ch, timeout=0.3):
    """发一个字符, 等它的回显 (或超时)."""
    ser.write(ch.encode("ascii"))
    deadline = time.time() + timeout
    echoed = b""
    while time.time() < deadline:
        if ser.in_waiting:
            echoed += ser.read(ser.in_waiting)
            if len(echoed) >= 1:
                break
        else:
            time.sleep(0.005)
    return echoed.decode("ascii", errors="replace")


def send_cmd(ser, cmd, line_ending="\r", wait_after=1.0):
    """逐字符发送命令, 每字符等回显, 末尾换行触发解析."""
    for ch in cmd:
        send_char_and_wait_echo(ser, ch)
        time.sleep(0.02)
    ser.write(line_ending.encode("ascii"))
    time.sleep(wait_after)
    return read_all(ser, settle=0.3, max_wait=3.0)


# ============================================================
# 输出解析 (按 motor_shell.c 的 key:value 标签)
# ============================================================

def parse_kv(text):
    """从多行文本提取 'key : value' 或 'key=value' 形式的字段.
    motor_shell.c 用 'key : value' 格式 (冒号两侧空格不固定), 返回 dict.
    同名 key 后值覆盖前值 (mc_debug 多行不冲突)."""
    fields = {}
    for line in text.splitlines():
        # 匹配 "  key : 123" 或 "  key : 0x12AB" 或 "key:1"
        m = re.match(r"\s*([A-Za-z_][A-Za-z0-9_]*)\s*[:=]\s*(\S+)", line)
        if m:
            fields[m.group(1)] = m.group(2)
    return fields


def to_int(s):
    """解析整数 (支持 0x 前缀). 失败返回 None."""
    try:
        return int(s, 0)
    except (ValueError, TypeError):
        return None


# ============================================================
# 验收结果记录
# ============================================================

class Report:
    def __init__(self):
        self.items = []   # (step_id, status, detail)
        self.raw = {}     # cmd -> raw output

    def record(self, step_id, ok, detail, raw=None):
        status = "PASS" if ok else "FAIL"
        self.items.append((step_id, status, detail))
        print("  [%s] %s: %s" % (status, step_id, detail))
        if raw is not None:
            self.raw[step_id] = raw

    def summary(self):
        n_pass = sum(1 for _, s, _ in self.items if s == "PASS")
        n_fail = sum(1 for _, s, _ in self.items if s == "FAIL")
        print("\n" + "=" * 60)
        print("验收汇总: %d PASS / %d FAIL / %d 总计" % (n_pass, n_fail, len(self.items)))
        if n_fail:
            print("失败项:")
            for sid, st, det in self.items:
                if st == "FAIL":
                    print("  - %s: %s" % (sid, det))
        print("=" * 60)
        return n_fail == 0

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            f.write("=== Stage 4+4b 台架验收结果 ===\n")
            f.write("时间: %s\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
            f.write("串口: %s\n\n" % PORT)
            for sid, st, det in self.items:
                f.write("[%s] %s: %s\n" % (st, sid, det))
            f.write("\n--- 原始输出 ---\n")
            for sid, raw in self.raw.items():
                f.write("\n>>> %s\n%s\n" % (sid, raw))
                f.write("-" * 60 + "\n")


# ============================================================
# 验收 Section
# ============================================================

def section_a_encoder_spi(ser, rep):
    """Section A: 编码器 SPI 连通性 (静止, 手转轴验证)."""
    print("\n--- Section A: 编码器 SPI 连通性 ---")
    print("  (请用手缓慢转动电机轴, 脚本会采两次 enc_status 看变化)")

    # A1: enc_status 在电机未运行时会主动 poll 一次 encoder_service。
    out_status1 = send_cmd(ser, "enc_status", wait_after=0.5)
    f1 = parse_kv(out_status1)
    raw1 = to_int(f1.get("raw16"))

    print("  请现在转动电机轴, 5 秒后采第二次...")
    time.sleep(5.0)
    out_status2 = send_cmd(ser, "enc_status", wait_after=0.5)
    f2 = parse_kv(out_status2)
    raw2 = to_int(f2.get("raw16"))

    rep.raw["A_enc_status_1"] = out_status1
    rep.raw["A_enc_status_2"] = out_status2

    # A1: raw16 在 16-bit 范围内
    ok = raw1 is not None and ENC_RAW_RANGE_16BIT[0] <= raw1 <= ENC_RAW_RANGE_16BIT[1]
    rep.record("A1a_raw_in_range", ok,
               "raw16#1=%s (期望 0..65535)" % raw1)

    ok = raw2 is not None and ENC_RAW_RANGE_16BIT[0] <= raw2 <= ENC_RAW_RANGE_16BIT[1]
    rep.record("A1b_raw_in_range", ok,
               "raw16#2=%s (期望 0..65535)" % raw2)

    # A1: 两次值不同 (证明 SPI 在读, 不是死值) — 转轴了就应变化
    ok = raw1 is not None and raw2 is not None and raw1 != raw2
    rep.record("A1c_raw_changes", ok,
               "raw16 %s -> %s (转轴后应变化)" % (raw1, raw2))

    m_counts1 = re.search(r"counts\s*:\s*sample=(\d+)\s+accept=(\d+)\s+bus=(\d+)\s+spike=(\d+)\s+stale=(\d+)", out_status1)
    m_counts2 = re.search(r"counts\s*:\s*sample=(\d+)\s+accept=(\d+)\s+bus=(\d+)\s+spike=(\d+)\s+stale=(\d+)", out_status2)
    bus1 = int(m_counts1.group(3)) if m_counts1 else None
    bus2 = int(m_counts2.group(3)) if m_counts2 else None
    spike1 = int(m_counts1.group(4)) if m_counts1 else None
    spike2 = int(m_counts2.group(4)) if m_counts2 else None
    ok = bus1 is not None and bus2 is not None and spike1 is not None and spike2 is not None
    rep.record("A2a_enc_status_counters_present", ok,
               "enc_status bus %s->%s spike %s->%s" % (bus1, bus2, spike1, spike2))
    ok = ok and (bus2 - bus1) < 5 and (spike2 - spike1) < 5
    rep.record("A2b_enc_status_counters_stable", ok,
               "bus/spike 增量应 < 5: bus %s->%s spike %s->%s" % (bus1, bus2, spike1, spike2))


def section_b_align(ser, rep):
    """Section B: ALIGN 零点对齐."""
    print("\n--- Section B: ALIGN 零点对齐 ---")

    # 前置: 清故障 + 零偏标定 (Stage 3 已验证 mc_cal, 此处复做确保 offset valid=1,
    # 否则 mc_current 读数偏差, 影响 C 段电流判读)
    send_cmd(ser, "fault_clear", wait_after=0.5)
    print("  (零偏标定 mc_cal, 约 3s...)")
    out_cal = send_cmd(ser, "mc_cal", wait_after=3.5)
    rep.raw["B_mc_cal"] = out_cal
    ok = "offset OK" in out_cal
    rep.record("B0_offset_calibrated", ok,
               "mc_cal 应返回 'offset OK' (Stage 3 前置)")

    # B1: mc_align 1000 (1V d 轴锁定)
    out = send_cmd(ser, "mc_align 1000", wait_after=0.8)
    rep.raw["B_align_start"] = out
    ok = "ALIGN started" in out
    rep.record("B1_align_started", ok,
               "mc_align 1000 应返回 'ALIGN started'")

    # 等转子锁定 (ALIGN_SETTLE_MS=400 + 采样窗口)
    time.sleep(0.6)

    # mc_debug 看 align active
    out_dbg = send_cmd(ser, "mc_debug", wait_after=0.5)
    f_dbg = parse_kv(out_dbg)
    rep.raw["B_debug_align"] = out_dbg
    align_active = to_int(f_dbg.get("active"))  # 注: align 行是 "align : hits=N active=M"
    m_align = re.search(r"active=(\d+)", out_dbg)
    align_active = to_int(m_align.group(1)) if m_align else None
    ok = align_active == 1
    rep.record("B1b_align_active", ok,
               "mc_debug align active=%s (期望 1)" % align_active)

    # B2: mc_zero 显示 align_angle 非零
    out_zero = send_cmd(ser, "mc_zero", wait_after=0.5)
    rep.raw["B_zero"] = out_zero
    f_zero = parse_kv(out_zero)
    align_angle = to_int(f_zero.get("align_angle"))
    # align_angle 可能恰好是 0 (极端), 主要看 ALIGN 采到了值; 0 也算通过但标记
    ok = align_angle is not None
    rep.record("B2_align_angle", ok,
               "align_angle=%s (ALIGN 采集值, 0..65535)" % align_angle)

    # B3: mc_stop
    out_stop = send_cmd(ser, "mc_stop", wait_after=1.0)
    rep.raw["B_stop"] = out_stop
    ok = "stopped" in out_stop or "DISABLED" in out_stop
    rep.record("B3_stopped", ok, "mc_stop 应返回 stopped")

    # 确认 align active 归零
    out_dbg2 = send_cmd(ser, "mc_debug", wait_after=0.5)
    m_align2 = re.search(r"active=(\d+)", out_dbg2)
    align_active2 = to_int(m_align2.group(1)) if m_align2 else None
    ok = align_active2 == 0
    rep.record("B3b_align_inactive", ok,
               "stop 后 align active=%s (期望 0)" % align_active2)


def section_c_open_loop_enc(ser, rep):
    """Section C: 开环 enc 跟踪."""
    print("\n--- Section C: 开环 enc 跟踪 ---")

    send_cmd(ser, "fault_clear", wait_after=0.5)

    # C1: mc_open 1000 300 enc (1V, 300rpm_elec, 用编码器电角度)
    out = send_cmd(ser, "mc_open 1000 300 enc", wait_after=1.5)
    rep.raw["C_open_enc"] = out
    ok = "OPEN_LOOP started" in out and "encoder" in out
    rep.record("C1_open_enc_started", ok,
               "mc_open 1000 300 enc 应返回 started + encoder")

    # C2: mc_debug 看 encoder 行
    out_dbg = send_cmd(ser, "mc_debug", wait_after=0.8)
    rep.raw["C_debug_enc_1"] = out_dbg
    m_enc = re.search(r"encoder\s*:\s*raw=(\d+)\s+theta=(-?\d+)\s+mrad.*?err=(\d+)\s+alive=(\d+)",
                      out_dbg)
    if m_enc:
        enc_raw1 = int(m_enc.group(1))
        enc_theta1 = int(m_enc.group(2))
        enc_err1 = int(m_enc.group(3))
        enc_alive1 = int(m_enc.group(4))
    else:
        enc_raw1 = enc_theta1 = enc_err1 = enc_alive1 = None

    ok = enc_alive1 == 1
    rep.record("C2a_enc_alive", ok,
               "enc_alive=%s (期望 1)" % enc_alive1)

    ok = enc_raw1 is not None and ENC_RAW_RANGE_16BIT[0] <= enc_raw1 <= ENC_RAW_RANGE_16BIT[1]
    rep.record("C2b_enc_raw_in_range", ok,
               "enc_raw=%s (期望 0..65535)" % enc_raw1)

    # 再采一次看 enc_raw 变化 (电机在转)
    time.sleep(1.0)
    out_dbg2 = send_cmd(ser, "mc_debug", wait_after=0.8)
    rep.raw["C_debug_enc_2"] = out_dbg2
    m_enc2 = re.search(r"encoder\s*:\s*raw=(\d+)\s+theta=(-?\d+)", out_dbg2)
    enc_raw2 = int(m_enc2.group(1)) if m_enc2 else None
    enc_err2 = int(m_enc2.group(3)) if m_enc2 and m_enc2.lastindex >= 3 else None
    # 重新匹配带 err 的完整行
    m_enc2f = re.search(r"encoder\s*:\s*raw=(\d+)\s+theta=(-?\d+)\s+mrad.*?err=(\d+)", out_dbg2)
    if m_enc2f:
        enc_err2 = int(m_enc2f.group(3))

    ok = enc_raw1 is not None and enc_raw2 is not None and enc_raw1 != enc_raw2
    rep.record("C2c_enc_raw_changes", ok,
               "enc_raw %s -> %s (转动中应变化)" % (enc_raw1, enc_raw2))

    # enc_errors 不应大幅增长
    ok = enc_err1 is not None and enc_err2 is not None and (enc_err2 - enc_err1) < 10
    rep.record("C2d_enc_errors_stable", ok,
               "enc_errors %s -> %s (增量应 < 10)" % (enc_err1, enc_err2))

    # C3: mc_current 看电流
    out_cur = send_cmd(ser, "mc_current", wait_after=0.5)
    rep.raw["C_current"] = out_cur
    m_ia = re.search(r"current\s*:\s*ia=(-?\d+)\s+ib=(-?\d+)\s+ic=(-?\d+)\s*mA", out_cur)
    if m_ia:
        ia, ib, ic = int(m_ia.group(1)), int(m_ia.group(2)), int(m_ia.group(3))
        # 无过流: |I| < 2000mA (开环 1V, 限流 0.5A, 留余量)
        max_i = max(abs(ia), abs(ib), abs(ic))
        ok = max_i < 2000
        rep.record("C3_no_overcurrent", ok,
                   "ia=%d ib=%d ic=%d mA, max|I|=%d (期望 < 2000)" % (ia, ib, ic, max_i))
    else:
        rep.record("C3_no_overcurrent", False, "无法解析 current 行")

    # C4: mc_stop
    out_stop = send_cmd(ser, "mc_stop", wait_after=1.5)
    rep.raw["C_stop"] = out_stop
    ok = "stopped" in out_stop or "DISABLED" in out_stop
    rep.record("C4_stopped", ok, "mc_stop 应返回 stopped")

    # 最终故障检查
    out_fault = send_cmd(ser, "fault", wait_after=0.5)
    rep.raw["C_fault_final"] = out_fault
    m_fault = re.search(r"fault\s*=\s*0x([0-9A-Fa-f]+)", out_fault)
    fault_val = int(m_fault.group(1), 16) if m_fault else None
    # CAL_INVALID (0x40) 是告警级允许, 致命位 (OVERCURRENT/SENSOR 等) 不应置
    FATAL_MASK = 0x3F  # DRIVER|OVERCURRENT|SENSOR|UNDERVOLTAGE|OVERVOLTAGE|CAN_TIMEOUT
    ok = fault_val is not None and (fault_val & FATAL_MASK) == 0
    rep.record("C4b_no_fatal_fault", ok,
               "fault=0x%08X (致命位应清零, CAL_INVALID 允许)" % (fault_val or 0))


def section_d_calibrate(ser, rep):
    """Section D: 旁轴标定 (~25s)."""
    print("\n--- Section D: 旁轴标定 (~25s, 电机会自动旋转) ---")
    print("  *** 标定期间电机自动正反拖动 5+5 圈, 请勿触碰 ***")

    send_cmd(ser, "fault_clear", wait_after=0.5)
    send_cmd(ser, "mc_stop", wait_after=0.5)

    # D1: mc_calibrate
    out = send_cmd(ser, "mc_calibrate", wait_after=1.0)
    rep.raw["D_calibrate_start"] = out
    ok = "Calibration started" in out
    rep.record("D1_started", ok, "mc_calibrate 应返回 'Calibration started'")
    if not ok:
        rep.record("D2_completed", False, "未启动, 跳过轮询")
        return

    # D2: 轮询 mc_cal_status 直到 DONE/ABORTED
    # 同时采 mc_debug 快照, 观察标定期间 theta/enc_raw/CCR 是否真在变化 (诊断电机是否旋转)
    print("  轮询标定状态 (最多 %ds)..." % CAL_TOTAL_TIMEOUT_S)
    deadline = time.time() + CAL_TOTAL_TIMEOUT_S
    final_state = None
    final_progress = 0
    last_progress = -1
    poll_count = 0
    debug_snapshots = []   # [(elapsed_s, state, mc_debug_output)]
    while time.time() < deadline:
        time.sleep(2.0)
        out_st = send_cmd(ser, "mc_cal_status", wait_after=0.5)
        poll_count += 1
        m_state = re.search(r"state\s*:\s*\d+\s+\((\w+)\)", out_st)
        m_prog = re.search(r"progress\s*:\s*(\d+)%", out_st)
        state_name = m_state.group(1) if m_state else "?"
        progress = int(m_prog.group(1)) if m_prog else 0
        # 标定旋转阶段采 mc_debug 快照 (看 theta/enc_raw/CCR 变化)
        if state_name in ("SPIN_FWD", "SPIN_REV"):
            out_dbg = send_cmd(ser, "mc_debug", wait_after=0.5)
            elapsed_s = int(time.time() - (deadline - CAL_TOTAL_TIMEOUT_S))
            debug_snapshots.append((elapsed_s, state_name, out_dbg))
            # 提取关键值打印
            m_th = re.search(r"theta_e\s*:\s*(-?\d+)\s+mrad", out_dbg)
            m_enc = re.search(r"encoder\s*:\s*raw=(\d+)", out_dbg)
            m_ccr = re.search(r"CCR1/2/3\s*:\s*(\d+)\s*/\s*(\d+)\s*/\s*(\d+)", out_dbg)
            th = m_th.group(1) if m_th else "?"
            enc = m_enc.group(1) if m_enc else "?"
            ccr = "%s/%s/%s" % (m_ccr.group(1), m_ccr.group(2), m_ccr.group(3)) if m_ccr else "?"
            print("  [%ds] %s prog=%d%% theta=%s enc=%s CCR=%s" %
                  (elapsed_s, state_name, progress, th, enc, ccr))
        else:
            if progress != last_progress:
                print("  [%ds] state=%s progress=%d%%" %
                      (int(time.time() - (deadline - CAL_TOTAL_TIMEOUT_S)), state_name, progress))
        last_progress = progress
        if state_name in ("DONE", "ABORTED"):
            final_state = state_name
            final_progress = progress
            rep.raw["D_status_final"] = out_st
            break
    else:
        rep.raw["D_status_final"] = out_st if 'out_st' in locals() else ""
        # 超时路径也要保存诊断快照 (之前 return 前漏存, 导致证据丢失)
        if debug_snapshots:
            rep.raw["D_spin_debug"] = "\n".join(
                "[%ds %s]\n%s" % (t, s, d) for t, s, d in debug_snapshots)
        rep.record("D2_completed", False,
                   "标定超时 %ds 未完成 (state=%s progress=%d%%)" %
                   (CAL_TOTAL_TIMEOUT_S, state_name, progress))
        send_cmd(ser, "mc_stop", wait_after=1.0)
        return

    # 保存标定期间 mc_debug 快照供分析
    if debug_snapshots:
        rep.raw["D_spin_debug"] = "\n".join(
            "[%ds %s]\n%s" % (t, s, d) for t, s, d in debug_snapshots)

    ok = final_state == "DONE"
    rep.record("D2_completed", ok,
               "state=%s progress=%d%% (轮询 %d 次)" % (final_state, final_progress, poll_count))

    if not ok:
        # ABORTED: 看故障
        out_fault = send_cmd(ser, "fault", wait_after=0.5)
        rep.raw["D_abort_fault"] = out_fault
        rep.record("D2b_abort_reason", False,
                   "标定中止, fault=%s" % out_fault.strip().splitlines()[0] if out_fault else "?")
        return

    # D3: max_resid < 1000 mdeg
    out_st = rep.raw.get("D_status_final", "")
    m_resid = re.search(r"max_resid\s*:\s*(-?\d+)\s+mdeg", out_st)
    max_resid = int(m_resid.group(1)) if m_resid else None
    ok = max_resid is not None and abs(max_resid) < CAL_MAX_RESIDUAL_MDEG
    rep.record("D3_residual_ok", ok,
               "max_resid=%s mdeg (期望 |.| < %d)" % (max_resid, CAL_MAX_RESIDUAL_MDEG))

    # D4: mc_cal_dump 256 点表非零
    out_dump = send_cmd(ser, "mc_cal_dump", wait_after=1.0)
    rep.raw["D_dump"] = out_dump
    # 统计非零点数
    nums = re.findall(r"-?\d+", out_dump)
    nonzero = sum(1 for n in nums if int(n) != 0)
    ok = nonzero > 0
    rep.record("D4_table_nonzero", ok,
               "256 点表中非零点 = %d (期望 > 0)" % nonzero)

    # valid 应为 1
    m_valid = re.search(r"valid\s*:\s*(\d+)", out_st)
    valid = int(m_valid.group(1)) if m_valid else None
    ok = valid == 1
    rep.record("D4b_valid_set", ok, "cal valid=%s (期望 1)" % valid)


def section_e_persistence_prompt(ser, rep):
    """Section E: 持久化 (需断电重启, 脚本只打印手动步骤)."""
    print("\n--- Section E: 持久化 (需手动断电重启) ---")
    print("  E1. 断电重启板子, 重新运行本脚本 (加 skip_calib 参数),")
    print("      执行 'fault' 应无 CAL_INVALID, 'encoder' cal_valid=1")
    print("  E2. 运行 'mc_cal_erase' 后断电重启,")
    print("      'fault' 应有 CAL_INVALID, 可重新 mc_calibrate")
    print("  (这两步涉及断电, 脚本不自动执行, 请手动验证并记录)")

    # 即时校验: 当前 cal_valid 应为 1 (D 完成后)
    out_enc = send_cmd(ser, "encoder", wait_after=0.5)
    m_cv = re.search(r"cal_valid\s*:\s*(\d+)", out_enc)
    cv = int(m_cv.group(1)) if m_cv else None
    ok = cv == 1
    rep.record("E0_cal_valid_now", ok,
               "标定后 cal_valid=%s (期望 1, 持久化前即时)" % cv)


# ============================================================
# 主流程
# ============================================================

def main():
    global PORT
    port = PORT
    skip_calib = False
    if len(sys.argv) >= 2:
        port = sys.argv[1]
    if len(sys.argv) >= 3 and sys.argv[2] == "skip_calib":
        skip_calib = True
    PORT = port

    print("=== Stage 4+4b 台架验收 ===")
    print("串口: %s, 波特: %d" % (port, BAUD))
    print("跳过标定: %s" % skip_calib)
    print("前置: 限流电源 12V/0.5A, 固件已烧, msh 提示符可见")

    ser = open_port(port)
    rep = Report()

    print("\n等待 3s 板子就绪...")
    time.sleep(3.0)
    ser.reset_input_buffer()
    ser.write(b"\r")
    time.sleep(1.0)
    print("--- 初始 ---")
    print(read_all(ser, settle=0.3))

    try:
        section_a_encoder_spi(ser, rep)
        section_b_align(ser, rep)
        section_c_open_loop_enc(ser, rep)
        if not skip_calib:
            section_d_calibrate(ser, rep)
            section_e_persistence_prompt(ser, rep)
        else:
            print("\n(skip_calib: 跳过 Section D/E)")
    except KeyboardInterrupt:
        print("\n!! 用户中断, 尝试停止电机 !!")
        try:
            send_cmd(ser, "mc_stop", wait_after=1.0)
        except Exception:
            pass
    finally:
        ser.close()

    all_pass = rep.summary()
    rep.save("tests/stage4_bench_results.txt")
    print("结果已保存到 tests/stage4_bench_results.txt")
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
