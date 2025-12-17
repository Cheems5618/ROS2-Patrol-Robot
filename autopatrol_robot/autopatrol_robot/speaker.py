import rclpy
from rclpy.node import Node
from autopatrol_interfaces.srv import SpeechText
import espeakng

class Speaker(Node):
    def __init__(self):
        super().__init__('speaker')
        self.speech_service = self.create_service(
            SpeechText,
            'speak_text',
            self.speech_text_callback
        )
        # 修正这里
        self.speaker_ = espeakng.Speaker()
        self.speaker_.voice = 'zh'  # 设置为中文女声

    def speech_text_callback(self, request, response):
        text = request.text
        self.get_logger().info(f'接受文本准备朗读: "{text}"')
        self.speaker_.say(text)
        self.speaker_.wait()
        response.result = True
        return response
    
    def speech_text(self, text):
        """调用服务合成语音 """

def main(args=None):
    rclpy.init(args=args)
    node = Speaker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
