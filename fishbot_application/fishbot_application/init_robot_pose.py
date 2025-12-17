from geometry_msgs.msg import PoseStamped
# 导入 BasicNavigator 类, 用于导航功能
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

def main():
    # 初始化 ROS 2 Python 客户端库
    rclpy.init()
    
    # 创建 BasicNavigator 对象
    nav = BasicNavigator()

    # 等待导航堆栈完全启动
    nav.waitUntilNav2Active()

    # 创建并设置初始位姿
    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = nav.get_clock().now().to_msg()
    init_pose.pose.position.x = 0.0  # 设置初始位置 x 坐标
    init_pose.pose.position.y = 0.0  # 设置初始位置 y 坐标
    init_pose.pose.position.z = 0.0  # 设置初始位置 z 坐标
    init_pose.pose.orientation.x = 0.0
    init_pose.pose.orientation.y = 0.0
    init_pose.pose.orientation.z = 0.0
    init_pose.pose.orientation.w = 1.0  # 设置初始朝向（四元数）

    # 设置机器人的初始位姿
    nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active()# 等待导航堆栈完全启动

    rclpy.spin_once(nav)
    print("Initial robot pose has been set.")

    # 关闭 ROS 2 客户端库
    rclpy.shutdown()