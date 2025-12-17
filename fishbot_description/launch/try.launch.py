import launch
import launch_ros
#launch.launch_description_sources是ROS 2中用于描述和管理启动文件的模块。
import launch.launch_description_sources
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #动作1,启动其他launch文件
    launch_path = os.path.join(get_package_share_directory('fishbot_description'), 'launch', 'display_robot.launch.py')
    action_include_launch = launch.actions.IncludeLaunchDescription(
        #IncludeLaunchDescription动作用于在当前的launch文件中包含和执行另一个launch文件。
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            #PythonLaunchDescriptionSource用于指定要包含的launch文件的路径，这里使用的是Python编写的launch文件。
            launch_path
        )
    )

    #动作2,打印数据
    action_print_log=launch.actions.LogInfo(
        #打印信息
        msg="Fishbot description launch文件正常启动."
    )

    #动作3,执行命令行
    action_topic_list = launch.actions.ExecuteProcess(
        cmd=['ros2', 'topic', 'list'],
        output='screen'
    )

    #动作4,组织动作成组，把多个动作放到一组
    action_group = launch.actions.GroupAction(
        [
            #动作5,定时器
            launch.actions.TimerAction(
                period=5.0,
                actions=[action_topic_list]
            ),
            launch.actions.TimerAction(
                period=2.0,
                actions=[action_include_launch]
            ),
            launch.actions.TimerAction(
                period=1.0,
                actions=[action_print_log]
            )
        ]
    )

    return launch.LaunchDescription([
        action_group
    ])