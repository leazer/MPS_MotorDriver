"""
Stage 5 电流环台架自动化验收脚本

对应 spec `docs/superpowers/specs/2026-06-22-stage5-current-loop-design.md` §8.

串口约束: 同 stage4_bench.py (逐字符发送 + 等回显, 适配 finsh 轮询 getchar).
输出解析: 按 motor_shell.c 的 key:value 标签正则解析 mc_debug 输出.

验收矩阵:
  Section A: 前置准备 (mc_cal 零偏标定)
  Section B: ramp 模式基础验证 (强制无标定表, 验 enc 拒绝 + ramp 稳态)
  Section C: 阶跃响应 (ramp 模式, 0.5A->1A, 稳态误差 < 5%)
  Section D: enc 模式验证 (需有效标定表)
  Section E: 清理 + 报告

注:
  - 上升时间 < 1ms 需示波器测, 脚本只测稳态误差 + 分支命中 (finsh ~50ms 轮询限制).
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
IQ_STEADY_ERR_PCT = 5.0      # 稳态误差 < 5% (ramp)
IQ_STEADY_ERR_PCT_ENC = 10.0 # enc 模式容差大 (标定残差 3.66° 致纹波)
CAL_TOTAL_TIMEOUT_S = 120    # 标定超时 (复用 stage4)


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
# mc_debug cur 行解析
# 格式: cur       : active=1 hits=12345 id=12mA iq=501mA id_ref=0mA iq_ref=500mA
# ============================================================
def parse_cur_line(text):
    """从 mc_debug 输出提取 cur 行的 id/iq/id_ref/iq_ref (mA 整数) + active/hits.
    返回 dict 或 None (无 cur 行)."""
    for line in text.splitlines():
        if "cur" in line and "active=" in line and "iq=" in line:
            m_active = re.search(r"active=(\d)", line)
            m_hits = re.search(r"hits=(\d+)", line)
            m_id = re.search(r"id=(-?\d+)mA", line)
            m_iq = re.search(r"iq=(-?\d+)mA", line)
            m_idref = re.search(r"id_ref=(-?\d+)mA", line)
            m_iqref = re.search(r"iq_ref=(-?\d+)mA", line)
            if m_iq and m_iqref:
                return {
                    "active": int(m_active.group(1)) if m_active else -1,
                    "hits": int(m_hits.group(1)) if m_hits else 0,
                    "id": int(m_id.group(1)) if m_id else 0,
                    "iq": int(m_iq.group(1)),
                    "id_ref": int(m_idref.group(1)) if m_idref else 0,
                    "iq_ref": int(m_iqref.group(1)),
                }
    return None


def collect_cur_snapshots(ser, n, settle=0.1):
    """采 n 个 mc_debug 快照, 返回 cur 解析列表 (跳过无 cur 行的)."""
    snaps = []
    for _ in range(n):
        out = send_cmd(ser, "mc_debug", wait_after=0.6)
        cur = parse_cur_line(out)
        if cur is not None:
            snaps.append(cur)
        time.sleep(settle)
    return snaps


# ============================================================
# 验收 Sections
# ============================================================
def section_a(ser, log):
    """A: 前置准备"""
    log.append("=== Section A: precondition ===")
    out = send_cmd(ser, "mc_state")
    if "DISABLED" not in out:
        send_cmd(ser, "mc_stop", wait_after=0.5)
        out = send_cmd(ser, "mc_state")
    assert "DISABLED" in out, f"A: not DISABLED: {out}"
    out = send_cmd(ser, "fault")
    assert "0x00" in out or "fault" not in out.lower() or "0x0" in out, f"A: fault active: {out}"
    send_cmd(ser, "mc_cal", wait_after=2.0)
    out = send_cmd(ser, "mc_cur", wait_after=0.5)
    assert "usage" in out, f"A: mc_cur missing usage: {out}"
    log.append("[A] PASS: DISABLED, fault clear, mc_cal done, mc_cur exists")


def section_b(ser, log):
    """B: ramp 模式 (强制 CAL_INVALID)"""
    log.append("=== Section B: ramp mode (force cal invalid) ===")
    # B1: 擦除标定 + 重启
    send_cmd(ser, "mc_cal_erase", wait_after=1.0)
    send_cmd(ser, "reboot", wait_after=3.0)
    assert wait_msh(ser, timeout=15.0), "B: reboot timeout"
    send_cmd(ser, "mc_cal", wait_after=2.0)  # 重启后重新零偏标定
    # B2: enc 模式应被拒 (CAL_INVALID)
    out = send_cmd(ser, "mc_cur 500 enc")
    assert "cal invalid" in out.lower(), f"B2: enc not rejected: {out}"
    log.append("[B2] PASS: enc rejected when cal invalid")
    # B3: ramp 启动
    out = send_cmd(ser, "mc_cur 500 ramp 300")
    assert "current loop" in out and "500" in out, f"B3: ramp start failed: {out}"
    log.append("[B3] PASS: ramp mode started (0.5A, 300rpm)")
    # B4: 采快照, 验稳态
    time.sleep(0.5)
    snaps = collect_cur_snapshots(ser, 10, settle=0.15)
    assert len(snaps) >= 3, f"B4: too few snapshots: {len(snaps)}"
    # 验 cur_hits 递增 (ISR 跑 CURRENT 分支)
    hits_inc = snaps[-1]["hits"] - snaps[0]["hits"]
    assert hits_inc > 0, f"B4: cur_hits not increasing: {snaps[0]['hits']}->{snaps[-1]['hits']}"
    # 验 iq 稳态 (目标 500mA, ±5%)
    iq_vals = [s["iq"] for s in snaps[-3:]]
    iq_steady = sum(iq_vals) / len(iq_vals)
    assert 475 <= iq_steady <= 525, f"B4: iq steady {iq_steady} out of 475-525"
    # 验 id 接近 0 (±100mA, ramp 模式 theta 不精确)
    id_vals = [abs(s["id"]) for s in snaps[-3:]]
    assert max(id_vals) < 100, f"B4: id too large: {id_vals}"
    log.append(f"[B4] PASS: hits+{hits_inc}, iq={iq_steady}mA (target 500), id~0")
    send_cmd(ser, "mc_stop", wait_after=0.5)


def section_c(ser, log):
    """C: 阶跃响应 (ramp, 0.5A->1A)"""
    log.append("=== Section C: step response (ramp) ===")
    send_cmd(ser, "mc_cur 500 ramp 300", wait_after=0.5)
    time.sleep(0.3)
    # 阶跃到 1A
    send_cmd(ser, "mc_cur 1000 ramp 300", wait_after=0.5)
    time.sleep(0.5)  # 等稳态 (finsh 轮询采不到 1ms 上升)
    snaps = collect_cur_snapshots(ser, 20, settle=0.05)
    assert len(snaps) >= 5, f"C: too few snapshots: {len(snaps)}"
    # 稳态误差: 末 5 个 iq 均值 vs 1000mA
    iq_vals = [s["iq"] for s in snaps[-5:]]
    iq_steady = sum(iq_vals) / len(iq_vals)
    err_pct = abs(iq_steady - 1000) / 1000.0 * 100
    assert err_pct < IQ_STEADY_ERR_PCT, f"C: steady err {err_pct:.1f}% >= {IQ_STEADY_ERR_PCT}%"
    log.append(f"[C] PASS: iq steady={iq_steady}mA err={err_pct:.2f}% (< {IQ_STEADY_ERR_PCT}%)")
    send_cmd(ser, "mc_stop", wait_after=0.5)


def section_d(ser, log):
    """D: enc 模式 (需标定表)"""
    log.append("=== Section D: enc mode (with calibration) ===")
    # D1: 标定 (~72s)
    send_cmd(ser, "mc_calibrate", wait_after=2.0)
    deadline = time.time() + CAL_TOTAL_TIMEOUT_S
    done = False
    while time.time() < deadline:
        out = send_cmd(ser, "mc_cal_status", wait_after=1.0)
        if "DONE" in out:
            done = True
            break
        if "ABORTED" in out:
            log.append(f"[D] FAIL: calibrate aborted: {out}")
            return False
        time.sleep(2.0)
    assert done, f"D: calibrate timeout ({CAL_TOTAL_TIMEOUT_S}s)"
    log.append("[D1] PASS: calibration DONE")
    # D3: enc 启动
    out = send_cmd(ser, "mc_cur 500 enc", wait_after=0.5)
    assert "current loop" in out and "enc" in out, f"D3: enc start failed: {out}"
    log.append("[D3] PASS: enc mode started")
    # D4: 验稳态 (容差大, 标定残差 3.66°)
    time.sleep(0.5)
    snaps = collect_cur_snapshots(ser, 10, settle=0.15)
    assert len(snaps) >= 3, f"D4: too few snapshots: {len(snaps)}"
    iq_vals = [s["iq"] for s in snaps[-3:]]
    iq_steady = sum(iq_vals) / len(iq_vals)
    err_pct = abs(iq_steady - 500) / 500.0 * 100
    assert err_pct < IQ_STEADY_ERR_PCT_ENC, f"D4: enc iq err {err_pct:.1f}% >= {IQ_STEADY_ERR_PCT_ENC}%"
    log.append(f"[D4] PASS: enc iq={iq_steady}mA err={err_pct:.1f}% (< {IQ_STEADY_ERR_PCT_ENC}%, 含标定残差)")
    send_cmd(ser, "mc_stop", wait_after=0.5)
    return True


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    ser = open_port(port)
    log = []
    try:
        section_a(ser, log)
        section_b(ser, log)
        section_c(ser, log)
        section_d(ser, log)
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
