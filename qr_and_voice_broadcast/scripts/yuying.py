import argparse
import time

import serial
from serial.tools import list_ports


RESPONSE_NAMES = {
    0x41: "text accepted",
    0x45: "receive/synthesis error",
    0x4A: "module initialized",
    0x4F: "idle/playback finished",
}

TEXT_ENCODINGS = {
    "gbk": ("gbk", 0x01),
    "gb2312": ("gb2312", 0x00),
    "utf-8": ("utf-8", 0x05),
}


class VTX316:
    """HS-S77-PL语音模块(VTX316芯片)驱动类，适配RDK X5"""

    def __init__(self, port='/dev/ttyS1', baudrate=115200, timeout=1):
        """初始化串口和模块"""
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(0.2)  # 等待模块初始化
        print(f"VTX316语音模块初始化完成: {port}, {baudrate}bps")

    def _send_bytes(self, data: bytes):
        """发送原始字节数据"""
        self.ser.write(data)
        self.ser.flush()
        time.sleep(0.002)  # 短暂延时保证发送完整

    def _build_frame(self, command: int, payload: bytes = b'', param: int | None = None) -> bytes:
        """Build a VTX316 command frame: FD + 2-byte length + command data."""
        data = bytes([command])
        if param is not None:
            data += bytes([param])
        data += payload

        if len(data) > 0xFFFF:
            raise ValueError("VTX316帧数据太长")

        return bytes([0xFD, (len(data) >> 8) & 0xFF, len(data) & 0xFF]) + data

    def _send_frame(self, command: int, payload: bytes = b'', param: int | None = None):
        frame = self._build_frame(command, payload, param)
        self._send_bytes(frame)
        print("发送帧:", frame.hex(" ").upper())

    def read_feedback(self, timeout=0.3) -> bytes:
        """Read returned status bytes such as 41, 45, 4A or 4F."""
        old_timeout = self.ser.timeout
        self.ser.timeout = timeout
        try:
            data = self.ser.read(32)
        finally:
            self.ser.timeout = old_timeout

        if data:
            names = [RESPONSE_NAMES.get(byte, "unknown") for byte in data]
            print("收到回传:", data.hex(" ").upper(), names)
        else:
            print("未收到回传；如果TX/RX没有双向连接，但有声音，也可以忽略。")
        return data

    def bobo(self, message: str, encoding='gbk', read_feedback=True):
        """文本播报（核心功能）"""
        if not message:
            return
        try:
            codec, encoding_param = TEXT_ENCODINGS[encoding]
            msg_bytes = message.encode(codec)
            self._send_frame(0x01, msg_bytes, encoding_param)
            if read_feedback:
                self.read_feedback()
            time.sleep(0.05 + (len(msg_bytes) * 0.001))  # 按文本长度延时
            print(f"已发送播报指令：{message}")
        except Exception as e:
            print(f"播报错误：{e}")

    def set_volume(self, level: int):
        """设置音量（0-10级，默认6级）"""
        level = max(0, min(10, level))  # 限制范围
        volume_code = chr(0x30 + level)
        self._send_frame(0x01, f"[v{volume_code}]".encode('gbk'), 0x01)
        self.read_feedback()
        time.sleep(0.1)
        print(f"音量已设置为 {level} 级")

    def stop(self):
        """停止当前语音播放"""
        self._send_frame(0x03)
        print("已停止语音播放")

    def resume(self):
        """恢复暂停的语音播放"""
        self._send_frame(0x04)
        print("已恢复语音播放")

    def set_speaker(self, idx: int):
        """设置发言人（1-7号，默认1号）"""
        idx = max(1, min(7, idx))
        cmd_str = f"[m5{idx}]"
        self._send_frame(0x01, cmd_str.encode('gbk'), 0x01)
        self.read_feedback()
        time.sleep(0.05)
        print(f"发言人已设置为 {idx} 号")

    def close(self):
        """关闭串口"""
        self.ser.close()
        print("串口已关闭")


def print_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("未发现串口设备。请检查USB转串口是否插入，或板载UART是否启用。")
        return

    for port in ports:
        print(f"{port.device}\t{port.description}\t{port.hwid}")


def loopback_test(port, baudrate, count=5):
    """Test UART TX/RX by shorting the board TX and RX pins together."""
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
    except serial.SerialException as exc:
        print(f"打开串口失败: {port}: {exc}")
        print("RDK X5 的 40pin 8/10 脚默认通常对应 /dev/ttyS1；也可以先运行 --list-ports 查看。")
        return

    print(f"开始回环测试: {port}, {baudrate}bps")
    print("请确认已经临时短接开发板物理 8 脚 TXD 和 10 脚 RXD。")
    try:
        for index in range(count):
            payload = f"AA55-{index}".encode("ascii")
            ser.reset_input_buffer()
            ser.write(payload)
            ser.flush()
            received = ser.read(len(payload))
            print(f"Send: {payload.decode('ascii')}  Recv: {received.decode('ascii', errors='replace')}")
            time.sleep(0.5)
    finally:
        ser.close()


def main(argv=None):
    """Run a simple hardware smoke test for the voice module."""
    parser = argparse.ArgumentParser(description="VTX316 voice module test tool")
    parser.add_argument("--port", default="/dev/ttyS1", help="串口设备，例如 /dev/ttyS1 或 /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200, help="串口波特率")
    parser.add_argument("--text", default="欢迎使用宇音天下研发的语音合成芯片", help="要播报的文本")
    parser.add_argument("--volume", type=int, default=10, help="音量，0-10")
    parser.add_argument("--speaker", type=int, help="发音人编号，1-7")
    parser.add_argument("--encoding", choices=sorted(TEXT_ENCODINGS), default="gbk", help="文本编码")
    parser.add_argument("--list-ports", action="store_true", help="列出当前可见串口后退出")
    parser.add_argument("--loopback", action="store_true", help="短接 40pin 8/10 脚后测试 UART 回环")
    parser.add_argument("--count", type=int, default=5, help="回环测试发送次数")
    args = parser.parse_args(argv)

    if args.list_ports:
        print_ports()
        return

    if args.loopback:
        loopback_test(args.port, args.baudrate, args.count)
        return

    try:
        voice = VTX316(port=args.port, baudrate=args.baudrate)
    except serial.SerialException as exc:
        print(f"打开串口失败: {args.port}: {exc}")
        print("请确认设备名是否正确，RDK X5 40pin 8/10 脚默认通常是 /dev/ttyS1。")
        print("可以先运行: ros2 run yuying_module yuying --list-ports")
        return

    try:
        voice.set_volume(args.volume)
        if args.speaker is not None:
            voice.set_speaker(args.speaker)
        voice.bobo(args.text, encoding=args.encoding)

    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        voice.close()


if __name__ == "__main__":
    main()
