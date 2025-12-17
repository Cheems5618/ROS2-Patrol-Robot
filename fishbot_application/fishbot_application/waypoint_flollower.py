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
    goal_poses = []
    go_pose1 = PoseStamped()
    go_pose1.header.frame_id = 'map'
    go_pose1.header.stamp = nav.get_clock().now().to_msg()
    go_pose1.pose.position.x = -2.0  # 设置初始位置 x 坐标
    go_pose1.pose.position.y = 1.0  # 设置初始位置 y 坐标
    go_pose1.pose.position.z = 0.0  # 设置初始位置 z 坐标
    go_pose1.pose.orientation.x = 0.0
    go_pose1.pose.orientation.y = 0.0
    go_pose1.pose.orientation.z = 0.0
    go_pose1.pose.orientation.w = 1.0  # 设置初始朝向（四元数）
    goal_poses.append(go_pose1)

    go_pose2 = PoseStamped()
    go_pose2.header.frame_id = 'map'
    go_pose2.header.stamp = nav.get_clock().now().to_msg()
    go_pose2.pose.position.x = 0.0  # 设置初始位置 x 坐标
    go_pose2.pose.position.y = 3.0  # 设置初始位置 y 坐标
    go_pose2.pose.position.z = 0.0  # 设置初始位置 z 坐标
    go_pose2.pose.orientation.x = 0.0
    go_pose2.pose.orientation.y = 0.0
    go_pose2.pose.orientation.z = 0.0
    go_pose2.pose.orientation.w = 1.0  # 设置初始朝向（四元数）
    goal_poses.append(go_pose2)

    go_pose3 = PoseStamped()
    go_pose3.header.frame_id = 'map'
    go_pose3.header.stamp = nav.get_clock().now().to_msg()
    go_pose3.pose.position.x = -4.0  # 设置初始位置 x 坐标
    go_pose3.pose.position.y = 3.5  # 设置初始位置 y 坐标
    go_pose3.pose.position.z = 0.0  # 设置初始位置 z 坐标
    go_pose3.pose.orientation.x = 0.0
    go_pose3.pose.orientation.y = 0.0
    go_pose3.pose.orientation.z = 0.0
    go_pose3.pose.orientation.w = 1.0  # 设置初始朝向（四元数）
    goal_poses.append(go_pose3)

    # 设置机器人的路点位姿
    nav.followWaypoints(goal_poses)

    # 等待机器人到达目标位姿,isTaskComplete() 方法用于检查导航任务是否完成,返回 True 或 False
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
             nav.get_logger().info(
            f"路点编号: {feedback.current_waypoint}"
             )

    result = nav.getResult()
    nav.get_logger().info(f"导航结果: {result}")

    # 关闭 ROS 2 客户端库
    rclpy.shutdown()