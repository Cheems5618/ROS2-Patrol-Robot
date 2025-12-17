from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from autopatrol_interfaces.srv import SpeechText
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class PatrolNode(BasicNavigator):
    def __init__(self):
        # 初始化 BasicNavigator
        super().__init__(node_name="patrol_node")

        # ------------------ 参数 ------------------
        # 初始位置点列表，每个元素格式 "x,y,yaw"
        self.declare_parameter("initial_points", ["0.0,0.0,0.0"])
        # 巡逻目标点列表
        self.declare_parameter("target_points", [
            "2.0,0.0,0.0",
            "1.0,1.0,1.57"
        ])
        # 图像保存路径
        self.declare_parameter("img_save_path", "/home/cheems/chapt/chapt7/chapt7_ws/src/autopatrol_robot/saved_img")

        # 获取参数值
        self.initial_points = self.get_parameter("initial_points").value
        self.target_points = self.get_parameter("target_points").value
        self.img_save_path = self.get_parameter("img_save_path").value

        # ------------------ 图像处理 ------------------
        self.bridge = CvBridge()
        self.latest_image = None  # 存储最新图像
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',  # 摄像头话题
            self.image_callback,
            1  # 队列大小
        )

        # ------------------ TF ------------------
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)

        # ------------------ 语音服务客户端 ------------------
        self.speech_client = self.create_client(SpeechText, 'speak_text')

    # ------------------ 图像回调 ------------------
    def image_callback(self, msg):
        """订阅回调函数，更新最新图像"""
        self.latest_image = msg

    def save_current_image(self, point_idx):
        """保存当前图像，若未接收到图像则等待"""
        while self.latest_image is None and rclpy.ok():
            self.get_logger().info("等待接收图像...")
            rclpy.spin_once(self, timeout_sec=0.5)

        # 确保保存目录存在
        os.makedirs(self.img_save_path, exist_ok=True)

        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        save_path = os.path.join(self.img_save_path, f"image_point_{point_idx}.png")
        cv2.imwrite(save_path, cv_image)
        self.get_logger().info(f"图像已保存到: {save_path}")


    # ------------------ 工具函数 ------------------
    def make_pose(self, x, y, yaw) -> PoseStamped:
        """根据x, y, yaw生成PoseStamped"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)

        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def parse_points(self, points):
        """将字符串列表解析成PoseStamped列表"""
        poses = []
        for p in points:
            x, y, yaw = p.split(",")
            poses.append(self.make_pose(x, y, yaw))
        return poses

    # ------------------ 业务逻辑 ------------------
    def set_initial_pose(self):
        """设置机器人初始位姿"""
        poses = self.parse_points(self.initial_points)
        if poses:
            self.setInitialPose(poses[0])
            self.waitUntilNav2Active()
            self.get_logger().info("初始位姿已设置")

    def navigate_patrol(self):
        """巡逻导航，边导航边播报并保存图像"""
        goals = self.parse_points(self.target_points)

        for idx, goal in enumerate(goals):
            # 提取坐标，用于播报
            x, y, yaw = self.target_points[idx].split(",")

            # 播报目标点信息
            self.speech_text(f"正在前往第{idx+1}个巡逻点，坐标为x点{float(x):.1f}米，y点{float(y):.1f}米")

            # 开始导航
            self.get_logger().info(f"导航到第 {idx + 1}/{len(goals)} 个目标点")
            self.goToPose(goal)

            # 等待导航完成
            while not self.isTaskComplete():
                feedback = self.getFeedback()
                if feedback:
                    self.get_logger().info(
                        f"剩余距离: {feedback.distance_remaining:.2f} m, "
                        f"已导航时间: {feedback.navigation_time.sec} s"
                    )

            # 获取导航结果
            result = self.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"已经到达目标点 {idx + 1}")
                # 保存图像
                self.save_current_image(idx+1)
                # 播报到达信息
                self.speech_text(f"已到达第{idx+1}个巡逻点，图像已保存")
            else:
                self.get_logger().warn("导航失败，停止巡航")
                break

    def get_current_pose(self):
        """获取机器人当前位姿"""
        try:
            tf = self.buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time(),
                rclpy.time.Duration(seconds=1)
            )
            yaw = euler_from_quaternion([
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w
            ])[2]
            return tf.transform.translation.x, tf.transform.translation.y, yaw
        except Exception as e:
            self.get_logger().warn(f"获取位姿失败: {e}")
            return None

    def speech_text(self, text):
        """调用语音服务进行播报"""
        while not self.speech_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待语音服务可用...')
        request = SpeechText.Request()
        request.text = text
        future = self.speech_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('语音合成完成')
        else:
            self.get_logger().error('语音合成服务调用失败')


# ------------------ 程序入口 ------------------
def main():
    rclpy.init()
    node = PatrolNode()
    node.waitUntilNav2Active()
    node.speech_text("机器人巡逻任务开始")
    node.set_initial_pose()
    node.speech_text("初始位置设置完成，开始巡逻")
    node.navigate_patrol()  # 边导航边播报、保存图像
    node.speech_text("巡逻任务结束")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
