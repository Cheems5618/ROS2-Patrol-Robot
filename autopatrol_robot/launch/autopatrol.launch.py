import launch   
import launch_ros
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch.actions
import launch.launch_description_sources
import os
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    # 获取配置文件路径
    autopatrol_robot_dir = get_package_share_directory('autopatrol_robot')
    default_autopatrol_config_path = os.path.join(
        autopatrol_robot_dir, 'config', 'patrol_config.yaml'
    )   


    # 创建autopatrol节点,用于巡逻，参数传入配置文件路径
    autopatrol_patrol_node = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='patrol_node',
        name='patrol_node',
        output='screen',
        parameters=[default_autopatrol_config_path]
    )
    # 创建Speaker节点,用于语音播报
    autopatrol_speaker_node = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='speaker',
        name='speaker', 
        output='screen'
    )





    return launch.LaunchDescription([
        autopatrol_patrol_node,
        autopatrol_speaker_node,
    ])