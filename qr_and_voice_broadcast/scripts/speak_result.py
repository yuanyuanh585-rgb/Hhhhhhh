import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import TransformStamped

from tf2_ros import Buffer, TransformListener

from yuying import VTX316


# 到达目标点的距离阈值（米）
DISTANCE_THRESHOLD = 0.3

# 需要触发图生文的目标点位
CAPTURE_WAYPOINTS = [
    {"x": 3.171748, "y": 4.217791},
    {"x": 0.850306, "y": 4.217791},
]


class SpeakResultNode(Node):
    """语音播报节点：
    - 订阅 /number（二维码解码结果），根据奇偶播报方向
    - 监听 TF 获取机器人实时位姿，到达目标点时触发图生文
    - 订阅 /image_result（图生文结果），播报出来
    """

    def __init__(self):
        super().__init__("speak_result_node")

        self.declare_parameter("serial_port", "/dev/ttyS1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("volume", 10)

        port = self.get_parameter("serial_port").value
        baudrate = self.get_parameter("baudrate").value
        volume = self.get_parameter("volume").value

        self.voice = VTX316(port=port, baudrate=baudrate)
        self.voice.set_volume(volume)
        self.get_logger().info(f"语音模块就绪: {port}")

        # --- 二维码播报 ---
        self.last_qr_number = None  # 防重：相同数字只播一次
        self.number_sub = self.create_subscription(
            String, "/number", self.on_qr_result, 10)
        self.get_logger().info("等待 /number (二维码解码结果) ...")

        # --- 图生文触发 ---
        self.trigger_pub = self.create_publisher(
            Int32, "/caption_trigger", 10)

        # --- 图生文结果播报 ---
        self.caption_sub = self.create_subscription(
            String, "/image_result", self.on_caption_result, 10)
        self.get_logger().info("等待 /image_result (图生文结果) ...")

        # --- TF 监听，检测是否到达目标点 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.triggered_set = set()  # 已触发过的点位索引，防重入

        self.pose_timer = self.create_timer(0.5, self.check_pose)

    def on_qr_result(self, msg: String):
        data_str = msg.data.strip()
        if not data_str:
            return

        try:
            number = int(data_str)
        except ValueError:
            self.get_logger().warn(f"无法解析为数字: '{data_str}'")
            return

        # 相同数字只播一次，避免每帧重复播报
        if number == self.last_qr_number:
            return
        self.last_qr_number = number

        if number % 2 != 0:
            text = "顺时针"
        else:
            text = "逆时针"

        self.get_logger().info(f"二维码数字: {number} → 播报: {text}")
        self.voice.bobo(text, encoding="gbk")

    def on_caption_result(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        # 去掉编号列表（如 "1. xxx\n2. xxx"）取第一句
        import re
        text = re.sub(r'^\d+[\.\、\s]+', '', text, flags=re.MULTILINE)
        # 取某一句，优先用换行分隔
        first_line = text.split('\n')[0].strip().rstrip('，。,')
        # 控制在 15 字以内
        if len(first_line) > 15:
            first_line = first_line[:15]
        self.get_logger().info(f"图生文结果播报: {first_line}")
        self.voice.bobo(first_line, encoding="gbk")

    def check_pose(self):
        """定时查询 TF，检测机器人是否到达目标点位"""
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time())
        except Exception as e:
            self.get_logger().debug(f"TF 查询失败: {e}")
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        for i, wp in enumerate(CAPTURE_WAYPOINTS):
            if i in self.triggered_set:
                continue
            dist = math.hypot(robot_x - wp["x"], robot_y - wp["y"])
            if dist < DISTANCE_THRESHOLD:
                self.get_logger().info(
                    f"到达目标点 ({wp['x']:.3f}, {wp['y']:.3f})，距离 {dist:.3f}m，触发图生文")
                trigger_msg = Int32()
                trigger_msg.data = 1
                self.trigger_pub.publish(trigger_msg)
                self.triggered_set.add(i)
                break


def main():
    rclpy.init()
    node = SpeakResultNode()
    try:
        rclpy.spin(node)
    finally:
        node.voice.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
