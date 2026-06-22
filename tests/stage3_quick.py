"""Stage 3 快速验证 - ADC 注入序列 + 零偏标定."""
import serial
import time

PORT = "COM9"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.5)
ser.reset_input_buffer()
ser.reset_output_buffer()

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

# 1. VBUS
print("\n>>> vbus")
print(send_cmd("vbus", wait_after=1.0))

# 2. 清故障 (CAL_INVALID)
print("\n>>> fault_clear")
print(send_cmd("fault_clear", wait_after=1.0))

# 3. 零偏标定
print("\n>>> mc_cal")
print(send_cmd("mc_cal", wait_after=3.0))

# 4. 标定后电流
print("\n>>> mc_current")
print(send_cmd("mc_current", wait_after=1.0))

# 5. mc_debug
print("\n>>> mc_debug")
print(send_cmd("mc_debug", wait_after=1.0))

# 6. 开环测试
print("\n>>> mc_open 1000 120")
print(send_cmd("mc_open 1000 120", wait_after=2.0))

# 7. 旋转中电流
print("\n>>> mc_current (旋转中)")
print(send_cmd("mc_current", wait_after=1.0))

# 8. 旋转中 debug
print("\n>>> mc_debug (旋转中)")
print(send_cmd("mc_debug", wait_after=1.0))

# 9. 停止
print("\n>>> mc_stop")
print(send_cmd("mc_stop", wait_after=1.5))

# 10. 最终状态
print("\n>>> mc_current (停止后)")
print(send_cmd("mc_current", wait_after=1.0))

print("\n>>> fault")
print(send_cmd("fault", wait_after=1.0))

ser.close()
print("\n=== done ===")
