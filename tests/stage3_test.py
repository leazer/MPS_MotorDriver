"""
COM9 测试 Stage 3 - ADC 电流采样与 VBUS 反馈验证

测试项目:
  1. vbus        - VBUS 母线电压读取 (软件触发普通转换)
  2. mc_cal      - 零偏标定 (PWM 50% 采 1024 次平均)
  3. mc_current  - 三相电流 + VBUS 详细 (零偏标定前后对比)
  4. mc_open     - 开环旋转, 验证电流读数随电机转动变化
  5. mc_debug    - ISR 内部状态 (含电流/VBUS 快照)
  6. mc_stop     - 停止, 验证电流归零

逐字符发送, 等待回显后再发下一个 (适配 finsh 轮询 getchar, 避免 overrun).
"""
import serial
import time

PORT = "COM9"
BAUD = 115200


def open_port():
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
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
    """发送一个字符, 等待它的回显 (或超时)."""
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
    """逐字符发送命令, 每字符等待回显, 末尾发换行符触发解析."""
    for ch in cmd:
        send_char_and_wait_echo(ser, ch)
        time.sleep(0.02)
    ser.write(line_ending.encode("ascii"))
    time.sleep(wait_after)
    return read_all(ser, settle=0.3, max_wait=3.0)


def main():
    ser = open_port()
    print("=== Stage 3 COM9 测试 ===")
    print("=== 等待 3s 板子启动 ===")
    time.sleep(3.0)
    ser.reset_input_buffer()

    # 触发提示符
    ser.write(b"\r")
    time.sleep(1.0)
    print("--- 初始 ---")
    print(read_all(ser, settle=0.3))

    results = {}

    # 1. 初始状态
    print("\n" + "=" * 60)
    print(">>> mc_state")
    results["mc_state_init"] = send_cmd(ser, "mc_state", wait_after=1.0)
    print(results["mc_state_init"])

    # 2. 初始故障 (应有 CAL_INVALID)
    print("\n>>> fault")
    results["fault_init"] = send_cmd(ser, "fault", wait_after=1.0)
    print(results["fault_init"])

    # 3. VBUS 读取 (软件触发, 不依赖 ISR)
    print("\n" + "=" * 60)
    print(">>> vbus (软件触发 VBUS 读取)")
    results["vbus_1"] = send_cmd(ser, "vbus", wait_after=1.0)
    print(results["vbus_1"])

    # 4. 标定前电流读数 (ISR 未启动, offset=2048 默认)
    print("\n>>> mc_current (标定前, offset=2048 默认)")
    results["current_pre_cal"] = send_cmd(ser, "mc_current", wait_after=1.0)
    print(results["current_pre_cal"])

    # 5. 零偏标定 (临时启动 ISR 让 ADC 注入序列跑, 标定后停止)
    print("\n" + "=" * 60)
    print(">>> mc_cal (零偏标定: PWM 50%, 1024 次平均)")
    results["mc_cal"] = send_cmd(ser, "mc_cal", wait_after=2.5)
    print(results["mc_cal"])

    # 6. 标定后电流读数 (offset 已更新)
    print("\n>>> mc_current (标定后, offset 已更新)")
    results["current_post_cal"] = send_cmd(ser, "mc_current", wait_after=1.0)
    print(results["current_post_cal"])

    # 7. mc_debug 完整状态
    print("\n>>> mc_debug (完整 ISR 状态)")
    results["mc_debug_1"] = send_cmd(ser, "mc_debug", wait_after=1.0)
    print(results["mc_debug_1"])

    # 8. 开环旋转测试 - 验证电流随电机转动变化
    print("\n" + "=" * 60)
    print(">>> mc_open 1000 120 (开环旋转: 1V, 120rpm_elec)")
    results["mc_open"] = send_cmd(ser, "mc_open 1000 120", wait_after=2.0)
    print(results["mc_open"])

    # 9. 旋转中读取电流 (应有非零电流)
    print("\n>>> mc_current (旋转中, 应有非零电流)")
    results["current_running"] = send_cmd(ser, "mc_current", wait_after=1.0)
    print(results["current_running"])

    # 10. 旋转中 mc_debug
    print("\n>>> mc_debug (旋转中)")
    results["mc_debug_running"] = send_cmd(ser, "mc_debug", wait_after=1.0)
    print(results["mc_debug_running"])

    # 11. 停止
    print("\n" + "=" * 60)
    print(">>> mc_stop")
    results["mc_stop"] = send_cmd(ser, "mc_stop", wait_after=1.5)
    print(results["mc_stop"])

    # 12. 停止后电流读数 (应归零)
    print("\n>>> mc_current (停止后, 电流应归零)")
    results["current_stopped"] = send_cmd(ser, "mc_current", wait_after=1.0)
    print(results["current_stopped"])

    # 13. 最终状态
    print("\n>>> mc_state")
    results["mc_state_final"] = send_cmd(ser, "mc_state", wait_after=1.0)
    print(results["mc_state_final"])

    print("\n>>> fault")
    results["fault_final"] = send_cmd(ser, "fault", wait_after=1.0)
    print(results["fault_final"])

    # 14. VBUS 再次读取 (验证一致性)
    print("\n>>> vbus (一致性检查)")
    results["vbus_2"] = send_cmd(ser, "vbus", wait_after=1.0)
    print(results["vbus_2"])

    ser.close()
    print("\n=== Stage 3 测试结束 ===")

    # 保存结果
    with open("tests/stage3_results.txt", "w", encoding="utf-8") as f:
        f.write("=== Stage 3 COM9 测试结果 ===\n")
        f.write("时间: %s\n\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
        for key, val in results.items():
            f.write(">>> %s\n" % key)
            f.write(val)
            f.write("\n" + "-" * 60 + "\n")

    print("\n结果已保存到 tests/stage3_results.txt")


if __name__ == "__main__":
    main()
