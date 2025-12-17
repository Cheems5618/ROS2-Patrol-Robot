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
    go_pose = PoseStamped()
    go_pose.header.frame_id = 'map'
    go_pose.header.stamp = nav.get_clock().now().to_msg()
    go_pose.pose.position.x = -2.0  # 设置初始位置 x 坐标
    go_pose.pose.position.y = 1.0  # 设置初始位置 y 坐标
    go_pose.pose.position.z = 0.0  # 设置初始位置 z 坐标
    go_pose.pose.orientation.x = 0.0
    go_pose.pose.orientation.y = 0.0
    go_pose.pose.orientation.z = 0.0
    go_pose.pose.orientation.w = 1.0  # 设置初始朝向（四元数）

    # 设置机器人的目标位姿
    nav.goToPose(go_pose)

    # 等待机器人到达目标位姿,isTaskComplete() 方法用于检查导航任务是否完成,返回 True 或 False
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
             nav.get_logger().info(
            f"距离目标还有: {feedback.distance_remaining:.2f} 米, "
            f"已导航时间: {feedback.navigation_time.sec} 秒"
             )

    result = nav.getResult()
    nav.get_logger().info(f"导航结果: {result}")

    # 关闭 ROS 2 客户端库
    rclpy.shutdown()