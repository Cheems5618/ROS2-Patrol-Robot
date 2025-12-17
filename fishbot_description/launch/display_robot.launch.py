#用于启动ros-$ROS_DISTRO-joint-state-publisher，发布关节状态信息
#用于启动ros-$ROS_DISTRO-robot-state-publisher，发布机器人状态信息
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_ros.parameter_descriptions
#获取包的共享目录,用于定位资源文件,如URDF文件

def generate_launch_description():
    # 获取 fishbot_description 包的 urdf 路径
    urdf_package_path = get_package_share_directory('fishbot_description')
    #urdf_path = os.path.join(urdf_package_path, 'urdf', 'first_robot.urdf')
    default_xacro_path = os.path.join(urdf_package_path, 'urdf', 'fishbot', 'fishbot.urdf.xacro')
    default_rviz_config_path = os.path.join(urdf_package_path, 'rviz', 'display_robot_model.rviz')

    # 使用 xacro 命令行工具将 xacro 文件转换为 urdf 内容

    robot_description_content = Command(['xacro  ', default_xacro_path])

    # 启动 robot_state_publisher 节点
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )


    # 启动 rviz2 节点
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path]
    )

    return launch.LaunchDescription([
        action_robot_state_publisher,
        action_rviz_node
    ])