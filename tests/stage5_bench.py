"""
Stage 5 电流环台架自动化验收脚本

对应 spec `docs/superpowers/specs/2026-06-22-stage5-current-loop-design.md` §8.

串口约束: 同 stage4_bench.py (逐字符发送 + 等回显, 适配 finsh 轮询 getchar).
输出解析: 按 motor_shell.c 的 key:value 标签正则解析 mc_debug 输出.

验收矩阵:
  Section A: 前置准备 (mc_cal 零偏标定)
  Full quadrant: 使用已有编码器标定, 验证 ±50/100/200/500mA

注:
  - 脚本不会擦除或自动执行编码器标定; 标定无效时会停止并提示先完成标定.
  - 自动测试电流命令限制在 ±500mA.
  - 命令名 mc_cur (与 mc_current 电流采样显示区分, 见 motor_shell.c).

用法:
  python tests/stage5_bench.py            # 默认 COM9
  python tests/stage5_bench.py COM7       # 指定串口
"""
import serial
import sys
import time
import re

PORT = "COM9"
BAUD = 115200

# ---- 验收阈值 ----
CURRENT_TEST_POINTS_MA = (-50, 50, -100, 100, -200, 200, -500, 500)


# ============================================================
# 串口基础 (复制自 stage4_bench.py)
# ============================================================
def open_port(port):
    ser = serial.Serial(port, BAUD, timeout=0.5)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def read_all(ser, settle=0.15, max_wait=2.0):
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
    for ch in cmd:
        send_char_and_wait_echo(ser, ch)
        time.sleep(0.02)
    ser.write(line_ending.encode("ascii"))
    time.sleep(wait_after)
    return read_all(ser, settle=0.3, max_wait=3.0)


def wait_msh(ser, timeout=15.0):
    """等 msh 提示符 (板子启动/重启后)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        data = read_all(ser, settle=0.5, max_wait=2.0)
        if "msh />" in data or "msh" in data:
            time.sleep(0.5)
            return True
        time.sleep(0.3)
    return False


# ============================================================
# mc_debug 电流与采样质量解析
# ============================================================
def parse_current_snapshot(text):
    snapshot = {}
    avg = re.search(r"cur_avg\s*:\s*id=(-?\d+)mA iq=(-?\d+)mA", text)
    count = re.search(
        r"sample_count:\s*invalid_total=(\d+) invalid_consecutive=(\d+) pi_freeze=(\d+)",
        text,
    )
    sample = re.search(r"sample\s*:\s*tick=(\d+) valid_mask=0x([0-9A-Fa-f]+) recon=(\d+)", text)
    if not avg or not count or not sample:
        return None
    snapshot["id_avg"] = int(avg.group(1))
    snapshot["iq_avg"] = int(avg.group(2))
    snapshot["invalid_total"] = int(count.group(1))
    snapshot["invalid_consecutive"] = int(count.group(2))
    snapshot["pi_freeze"] = int(count.group(3))
    snapshot["sample_tick"] = int(sample.group(1))
    snapshot["valid_mask"] = int(sample.group(2), 16)
    snapshot["recon"] = int(sample.group(3))
    return snapshot


def parse_encoder_calibration_valid(text):
    valid = re.search(r"(?m)^\s*valid\s*:\s*([01])\s*$", text)
    return valid is not None and valid.group(1) == "1"


def read_current_snapshot(ser):
    text = send_cmd(ser, "mc_debug", wait_after=0.6)
    snapshot = parse_current_snapshot(text)
    assert snapshot is not None, f"missing current/sample diagnostics: {text}"
    return snapshot


def read_fault_value(ser):
    text = send_cmd(ser, "fault", wait_after=0.2)
    match = re.search(r"fault\s*=\s*0x([0-9A-Fa-f]+)", text)
    assert match, f"missing fault value: {text}"
    return int(match.group(1), 16)


# ============================================================
# 验收 Sections
# ============================================================
def section_a(ser, log):
    """A: 前置准备. mc_state 输出 state : 0 (DISABLED=0/ENABLED=1/FAULT=2).
    fault 命令输出 'fault = 0xHHHHHHHH' (位掩码, 0=无故障).
    CAL_INVALID(0x40) 是告警级不阻止使能;
    致命故障 (含 CURRENT_SAMPLE, mask=0x9F) 必须先 fault_clear."""
    log.append("=== Section A: precondition ===")
    out = send_cmd(ser, "mc_state")
    if not re.search(r"state\s*:\s*0\b", out):
        send_cmd(ser, "mc_stop", wait_after=0.5)
        out = send_cmd(ser, "mc_state")
    assert re.search(r"state\s*:\s*0\b", out), f"A: not DISABLED: {out.strip()}"
    # 清除可能残留的致命故障 (上次调试遗留 OC/SENSOR 等)
    send_cmd(ser, "fault_clear", wait_after=0.3)
    out = send_cmd(ser, "fault")
    m_fault = re.search(r"fault\s*=\s*0x([0-9A-Fa-f]+)", out)
    fault_val = int(m_fault.group(1), 16) if m_fault else 0xFFFFFFFF
    # 致命掩码 0x9F (含 CURRENT_SAMPLE); CAL_INVALID(0x40) 可接受
    assert (fault_val & 0x9F) == 0, f"A: fatal fault active: {out.strip()}"
    send_cmd(ser, "mc_cal", wait_after=2.0)
    out = send_cmd(ser, "mc_cur", wait_after=0.5)
    assert "usage" in out, f"A: mc_cur missing usage: {out}"
    log.append("[A] PASS: DISABLED, fault clear, mc_cal done, mc_cur exists")


def section_full_quadrant_current(ser, log):
    status = send_cmd(ser, "enc_cal_status", wait_after=0.3)
    assert parse_encoder_calibration_valid(status), \
        "encoder calibration invalid; complete calibration before this test"
    send_cmd(ser, "fault_clear", wait_after=0.2)

    for target_ma in CURRENT_TEST_POINTS_MA:
        before = read_current_snapshot(ser)
        try:
            start = send_cmd(ser, f"mc_cur {target_ma} enc", wait_after=0.5)
            assert "current loop" in start, f"start failed at {target_ma}mA: {start}"
            time.sleep(0.5)
            snapshots = [read_current_snapshot(ser) for _ in range(3)]
            snapshot = snapshots[-1]
            tolerance_ma = max(20, abs(target_ma) * 0.10)
            assert abs(snapshot["iq_avg"] - target_ma) <= tolerance_ma
            assert abs(snapshot["id_avg"]) <= 100
            assert snapshot["invalid_consecutive"] == 0
            assert snapshot["invalid_total"] == before["invalid_total"]
            assert snapshot["pi_freeze"] == before["pi_freeze"]
            fault_value = read_fault_value(ser)
            assert (fault_value & 0x9F) == 0
            log.append(
                f"PASS {target_ma:+d}mA: id={snapshot['id_avg']}mA "
                f"iq={snapshot['iq_avg']}mA mask=0x{snapshot['valid_mask']:02X} "
                f"recon={snapshot['recon']}"
            )
        finally:
            send_cmd(ser, "mc_stop", wait_after=0.3)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    ser = open_port(port)
    log = []
    try:
        section_a(ser, log)
        section_full_quadrant_current(ser, log)
        log.append("\n=== ALL PASS ===")
    except AssertionError as e:
        log.append(f"\n=== FAIL: {e} ===")
    finally:
        send_cmd(ser, "mc_stop", wait_after=0.5)
        ser.close()
    report = "\n".join(log)
    print(report)
    with open("tests/stage5_bench_log.txt", "w", encoding="utf-8") as f:
        f.write(report)


if __name__ == "__main__":
    main()
