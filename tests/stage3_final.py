"""
Stage 3 最终验证 - ADC 电流采样 + VBUS + 零偏标定 + 开环电流观测

测试矩阵:
  1. vbus          - VBUS 母线电压 (3 次一致性)
  2. fault_clear   - 清初始 CAL_INVALID
  3. mc_cal        - 零偏标定 (应 PASS)
  4. mc_current    - 标定后静态电流 (应接近 0)
  5. mc_open 1000 60   - 低速旋转, 观察电流
  6. mc_open 2000 120  - 高压高速旋转, 观察电流增大
  7. mc_stop       - 停止, 电流归零
  8. fault         - 确认无故障
"""
import serial
import time

PORT = "COM9"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.5)
ser.reset_input_buffer()
ser.reset_output_buffer()

print("=== Stage 3 最终验证 ===")
print("等待 3s 板子启动...")
time.sleep(3.0)
ser.reset_input_buffer()

ser.write(b"\r")
time.sleep(1.0)

def read_all(settle=0.15, max_wait=2.0):
    data = b""
    deadline = time.time() + max_wait
    while time.time() < deadline:
        if ser.in_waiting:
            data += ser.read(ser.in_waiting)
            deadline = time.time() + settle
        else:
            time.sleep(0.01)
    return data.decode("ascii", errors="replace")

print("--- 初始 ---")
print(read_all())

def send_char_and_wait_echo(ch, timeout=0.3):
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

def send_cmd(cmd, wait_after=1.0):
    for ch in cmd:
        send_char_and_wait_echo(ch)
        time.sleep(0.02)
    ser.write(b"\r")
    time.sleep(wait_after)
    return read_all(settle=0.3, max_wait=3.0)

results = {}

# 1. 清故障
print("\n>>> fault_clear")
results["fault_clear"] = send_cmd("fault_clear", wait_after=1.0)
print(results["fault_clear"])

# 2. VBUS 三次读取 (一致性)
for i in range(3):
    print("\n>>> vbus (#%d)" % (i+1))
    results["vbus_%d" % (i+1)] = send_cmd("vbus", wait_after=1.0)
    print(results["vbus_%d" % (i+1)])

# 3. 零偏标定
print("\n>>> mc_cal (零偏标定)")
results["mc_cal"] = send_cmd("mc_cal", wait_after=3.0)
print(results["mc_cal"])

# 4. 标定后静态电流
print("\n>>> mc_current (标定后静态)")
results["current_static"] = send_cmd("mc_current", wait_after=1.0)
print(results["current_static"])

# 5. 低速旋转
print("\n>>> mc_open 1000 60 (1V, 60rpm_elec)")
results["mc_open_low"] = send_cmd("mc_open 1000 60", wait_after=2.0)
print(results["mc_open_low"])

print("\n>>> mc_current (低速旋转中)")
results["current_low"] = send_cmd("mc_current", wait_after=1.0)
print(results["current_low"])

print("\n>>> mc_debug (低速旋转中)")
results["debug_low"] = send_cmd("mc_debug", wait_after=1.0)
print(results["debug_low"])

# 6. 停止再高速
print("\n>>> mc_stop")
results["mc_stop_1"] = send_cmd("mc_stop", wait_after=1.5)
print(results["mc_stop_1"])

print("\n>>> mc_open 2000 120 (2V, 120rpm_elec)")
results["mc_open_high"] = send_cmd("mc_open 2000 120", wait_after=2.0)
print(results["mc_open_high"])

print("\n>>> mc_current (高速旋转中)")
results["current_high"] = send_cmd("mc_current", wait_after=1.0)
print(results["current_high"])

print("\n>>> mc_debug (高速旋转中)")
results["debug_high"] = send_cmd("mc_debug", wait_after=1.0)
print(results["debug_high"])

# 7. 停止
print("\n>>> mc_stop")
results["mc_stop_2"] = send_cmd("mc_stop", wait_after=1.5)
print(results["mc_stop_2"])

# 8. 最终状态
print("\n>>> mc_current (停止后)")
results["current_final"] = send_cmd("mc_current", wait_after=1.0)
print(results["current_final"])

print("\n>>> fault")
results["fault_final"] = send_cmd("fault", wait_after=1.0)
print(results["fault_final"])

print("\n>>> mc_state")
results["state_final"] = send_cmd("mc_state", wait_after=1.0)
print(results["state_final"])

ser.close()
print("\n=== Stage 3 最终验证结束 ===")

# 保存结果
with open("tests/stage3_final_results.txt", "w", encoding="utf-8") as f:
    f.write("=== Stage 3 最终验证结果 ===\n")
    f.write("时间: %s\n\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
    for key, val in results.items():
        f.write(">>> %s\n" % key)
        f.write(val)
        f.write("\n" + "-" * 60 + "\n")
print("结果已保存到 tests/stage3_final_results.txt")
