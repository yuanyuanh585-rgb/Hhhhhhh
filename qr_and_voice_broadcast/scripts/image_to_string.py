# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage  # 替换 hbm_img_msgs
# from std_msgs.msg import Int32, String
# from openai import OpenAI
# import base64
# import cv2
# import numpy as np
# from rclpy.qos import QoSProfile, ReliabilityPolicy

# # 配置豆包 API
# API_KEY = "342a2ba6-093b-485b-bab4-b4c6951892da"
# client = OpenAI(
#     base_url="https://ark.cn-beijing.volces.com/api/v3",
#     api_key=API_KEY,
# )

# #模型类型
# MODEL_ID = "doubao-1.5-vision-lite-250315"

# class ImageToTextNode(Node):
#     def __init__(self):
#         super().__init__("image_to_text_node")
#         # 订阅 /o_check 话题，类型为 std_msgs::msg::Int32
#         self.o_check_subscription = self.create_subscription(
#             Int32,
#             "/o_check",
#             self.o_check_callback,
#             10
#         )

#         #订阅压缩图片
#         self.subscription = self.create_subscription(
#             CompressedImage,      # 使用 CompressedImage
#             "/image",             # 订阅压缩图像话题
#             self.image_callback,
#             10                    # QoS
#         )
   
#         # 发布图生文结果到 image_result
#         self.result_publisher = self.create_publisher(String, "/image_result", 10)

#         self.get_logger().info("图生文节点启动，等待 /image 和 /o_check 事件")

#         self.is_ready = False  # 用于控制是否开始图生文

#     def o_check_callback(self, msg):
#         # 处理 /o_check 话题，触发图生文
#         if msg.data == 1:
#             self.is_ready = True
#             self.get_logger().info("收到启动信号，开始图生文")

#     def image_callback(self, msg):
#         if not self.is_ready:
#             return

#         try:
#             # 解码压缩图像数据
#             np_arr = np.frombuffer(msg.data, np.uint8)
#             img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
#             if img_bgr is None:
#                 raise ValueError("图像解码失败")

#             # 转成JPEG（用于base64编码）
#             _, buffer = cv2.imencode(".jpg", img_bgr)
#             img_base64 = base64.b64encode(buffer).decode("utf-8")

#             # 调用豆包图生文
#             self.get_logger().info("正在请求豆包图生文服务...")
#             response = client.chat.completions.create(
#                 model=MODEL_ID,
#                 messages=[
#                     {"role": "system", "content": "你是一个图像内容描述助手"},
#                     {
#                         "role": "user",
#                         "content": [
#                             {"type": "text", "text": "请描述图中卡片上的内容,大概40个字"},
#                             {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_base64}"}}
#                         ]
#                     }
#                 ]
#             )
#             result = response.choices[0].message.content
#             self.get_logger().info(f"图像描述结果：{result}")

#             # 发布结果
#             result_msg = String()
#             result_msg.data = result
#             self.result_publisher.publish(result_msg)

#             # 重置状态
#             self.is_ready = False

#         except Exception as e:
#             self.get_logger().error(f"图像处理失败: {e}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = ImageToTextNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()




# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from hbm_img_msgs.msg import HbmMsg1080P
# from std_msgs.msg import Int32, String
# from openai import OpenAI
# import base64
# import cv2
# import numpy as np
# from rclpy.qos import QoSProfile, ReliabilityPolicy

# # 配置豆包 API
# API_KEY = "b9713149-4cac-478c-8a55-8b5877a1000b"
# client = OpenAI(
#     base_url="https://ark.cn-beijing.volces.com/api/v3",
#     api_key=API_KEY,
# )
# #模型类型
# MODEL_ID = "doubao-1.5-vision-lite-250315"

# class ImageToTextNode(Node):
#     def __init__(self):
#         super().__init__("image_to_text_node")
#         # 订阅 /o_check 话题，类型为 std_msgs::msg::Int32
#         self.o_check_subscription = self.create_subscription(
#             Int32,
#             "/o_check",
#             self.o_check_callback,
#             10
#         )
#         # 订阅 /hbmem_img 图像数据
#         #更改Qos
#         qos_profile = QoSProfile(depth=10)
#         qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

#         self.subscription = self.create_subscription(
#             HbmMsg1080P,
#             "/hbmem_img",
#             self.image_callback,
#             qos_profile   # 使用自定义 QoSProfile
#         )
#         # 发布图生文结果到 image_result
#         self.result_publisher = self.create_publisher(String, "/image_result", 10)

#         self.get_logger().info("图生文节点启动，等待 /hbmem_img 和 /o_check 事件")

#         self.is_ready = False  # 用于控制是否开始图生文，false 代表未收到启动信号

#     def o_check_callback(self, msg):
#         # 处理 /o_check 话题，触发图生文
#         if msg.data == 1:
#             self.is_ready = True
#             self.get_logger().info("收到启动信号，开始图生文")

#     def image_callback(self, msg):
#         # 图像处理和图生文功能
#         if not self.is_ready:
#             return

#         try:
#             height = msg.height
#             width = msg.width
#             encoding_raw = list(msg.encoding)
#             data = msg.data

#             # 解码编码类型
#             encoding_str = bytes(encoding_raw).decode('ascii', errors='ignore').split('\x00')[0].lower()
#             self.get_logger().info(f"接收到图像：{width}x{height}, encoding={encoding_str}")

#             # 解析图像数据
#             img_data = np.frombuffer(data, dtype=np.uint8)

#             if encoding_str == 'nv12':
#                 expected_size = height * width * 3 // 2
#                 img_data = img_data[:expected_size]
#                 img_yuv = img_data.reshape((height * 3 // 2, width))
#                 img_bgr = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR_NV12)

#             elif encoding_str == 'rgb8':
#                 expected_size = height * width * 3
#                 img_data = img_data[:expected_size]
#                 img_rgb = img_data.reshape((height, width, 3))
#                 img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

#             elif encoding_str in ['mono8', 'gray']:
#                 expected_size = height * width
#                 img_bgr = img_data[:expected_size].reshape((height, width))

#             else:
#                 raise ValueError(f"不支持的图像编码类型: {encoding_str}")

#             # 转成JPEG
#             _, buffer = cv2.imencode(".jpg", img_bgr)
#             img_base64 = base64.b64encode(buffer).decode("utf-8")

#             # 调用豆包模型
#             self.get_logger().info("正在请求豆包图生文服务...")
#             response = client.chat.completions.create(
#                 model=MODEL_ID,
#                 messages=[
#                     {"role": "system", "content": "你是一个图像内容描述助手"},
#                     {
#                         "role": "user",
#                         "content": [
#                             {"type": "text", "text": "请简要描述这张图的内容,40个字左右"},
#                             {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_base64}"}}
#                         ]
#                     }
#                 ]
#             )
#             result = response.choices[0].message.content
#             self.get_logger().info(f"图像描述结果：{result}")

#             # 发布图生文结果到 image_result 话题
#             result_msg = String()
#             result_msg.data = result
#             self.result_publisher.publish(result_msg)

#             # 重置状态
#             self.is_ready = False

#         except Exception as e:
#             self.get_logger().error(f"图像处理失败: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ImageToTextNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, String
import base64
import cv2
import numpy as np
from urllib import request, error

# 配置本地 Ollama 视觉模型
OLLAMA_BASE_URL = "http://192.168.0.120:11434/v1"
MODEL_ID = "llava-fast:latest"
class ImageToTextNode(Node):
    def __init__(self):
        super().__init__("image_to_text_node")

        # 持续接收最新一帧图像（缓存起来，不处理）
        self.latest_image = None
        self.subscription = self.create_subscription(
            CompressedImage,
            "/image",
            self.image_callback,
            10
        )

        # 收到触发后才做一次图生文
        self.trigger_sub = self.create_subscription(
            Int32,
            "/caption_trigger",
            self.trigger_callback,
            10
        )

        # 发布图生文结果到 image_result
        self.result_publisher = self.create_publisher(String, "/image_result", 10)

        self.get_logger().info("图生文节点启动，等待 /caption_trigger ...")

    def image_callback(self, msg):
        """只缓存最新一帧，不做推理"""
        self.latest_image = msg

    def trigger_callback(self, msg):
        """收到触发信号，用最新缓存帧做一次图生文"""
        if self.latest_image is None:
            self.get_logger().warn("收到触发信号但尚无图像缓存，跳过")
            return

        try:
            np_arr = np.frombuffer(self.latest_image.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img_bgr is None:
                raise ValueError("图像解码失败")

            _, buffer = cv2.imencode(".jpg", img_bgr)
            img_base64 = base64.b64encode(buffer).decode("utf-8")

            self.get_logger().info("正在请求本地视觉模型...")
            client = OpenAI(base_url=OLLAMA_BASE_URL, api_key="ollama")
            response = client.chat.completions.create(
                model=MODEL_ID,
                max_tokens=60,
                temperature=0,
                messages=[{
                    "role": "user",
                    "content": [
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_base64}"}},
                        {"type": "text", "text": "请用一句简短的中文描述这张图片，不超过10个字。"},
                    ],
                }]
            )

            result = response.choices[0].message.content
            self.get_logger().info(f"图像描述结果：{result}")

            result_msg = String()
            result_msg.data = result
            self.result_publisher.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f"图像处理失败: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageToTextNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()