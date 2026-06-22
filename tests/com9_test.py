"""
COM9 测试 v6 - 逐字符发送, 等待回显后再发下一个 (模拟真实终端).
适配 finsh 轮询 getchar (board.c rt_hw_console_getchar), 避免 overrun 丢字符.
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
    # 等待回显
    deadline = time.time() + timeout
    echoed = b""
    while time.time() < deadline:
        if ser.in_waiting:
            echoed += ser.read(ser.in_waiting)
            # 收到回显字符就停
            if len(echoed) >= 1:
                break
        else:
            time.sleep(0.005)
    return echoed.decode("ascii", errors="replace")


def send_cmd(ser, cmd, line_ending="\r", wait_after=1.0):
    """逐字符发送命令, 每字符等待回显, 末尾发换行符触发解析."""
    for ch in cmd:
        send_char_and_wait_echo(ser, ch)
        time.sleep(0.02)  # 额外间隙, 给 finsh 线程处理时间
    # 发送行结束符
    ser.write(line_ending.encode("ascii"))
    time.sleep(wait_after)
    return read_all(ser, settle=0.3, max_wait=3.0)


def main():
    ser = open_port()
    # 等待板子启动
    print("=== 等待 3s 板子启动 ===")
    time.sleep(3.0)
    ser.reset_input_buffer()

    # 触发提示符
    ser.write(b"\r")
    time.sleep(1.0)
    print("--- 初始 ---")
    print(read_all(ser, settle=0.3))

    cmds = [
        "mc_state",
        "fault",
        "mc_debug",
        "mc_open 1000 60",
        "mc_debug",
        "mc_debug",
        "mc_stop",
        "mc_debug",
        "mc_state",
    ]

    for cmd in cmds:
        print("\n>>> " + cmd)
        resp = send_cmd(ser, cmd, line_ending="\r", wait_after=1.5)
        print(resp)
        print("-" * 60)

    ser.close()
    print("\n=== 测试结束 ===")


if __name__ == "__main__":
    main()
