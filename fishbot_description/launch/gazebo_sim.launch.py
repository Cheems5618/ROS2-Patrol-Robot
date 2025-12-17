#用于启动ros-$ROS_DISTRO-joint-state-publisher，发布关节状态信息
#用于启动ros-$ROS_DISTRO-robot-state-publisher，发布机器人状态信息
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command, PathJoinSubstitution
import launch.launch_description_sources
import launch.actions
#获取包的共享目录,用于定位资源文件,如URDF文件

def generate_launch_description():
    #urdf_package_path是fishbot_description包的共享目录路径
    urdf_package_path = get_package_share_directory('fishbot_description')
    #xacro_path是fishbot.urdf.xacro文件路径
    default_xacro_path = os.path.join(urdf_package_path, 'urdf', 'fishbot', 'fishbot.urdf.xacro')
    #default_gazebo_world_path是gazebo仿真世界文件路径
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world', 'custom_room.world')
    #default_rviz_config_path是rviz配置文件路径
    default_rviz_config_path = os.path.join(urdf_package_path, 'rviz', 'display_robot_model.rviz')

    # 使用 xacro 命令行工具将 xacro 文件转换为 urdf 内容
    robot_description_content = Command(
    'bash -c "ros2 run xacro xacro ' + default_xacro_path + '"')
    # 启动 robot_state_publisher 节点
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}]
    )

    # 启动 gazebo 仿真环境,IncludeLaunchDescription用于包含另一个launch文件,在库launch_ros.actions中
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        #PythonLaunchDescriptionSource用于加载Python格式的launch文件
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        # 传递启动参数
        launch_arguments=[
            ('world', default_gazebo_world_path),
            ('verbose', 'true')
        ]
    )

    # 启动 joint_state_publisher 节点
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # 启动实体生成节点,arguments参数指定从robot_description话题获取机器人描述并命名实体为fishbot
    #arguments参数包含这几个部分，-topic, 'robot_description', -entity, 'fishbot'分别对应话题和实体名称
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'fishbot'
        ],
        parameters=[{'use_sim_time': True}]
    )

    # 加载并启动 joint_state_controller 控制器
    action_load_joint_state_controller = launch.actions.ExecuteProcess(
    # 指令的意义是加载并激活名为 fishbot_joint_state_broadcaster 的控制器
    cmd=[
        'ros2', 'control', 'load_controller',
        'fishbot_joint_state_broadcaster',
        '--set-state', 'active'
    ],
    output='screen'
    )

    # 加载并启动 diff_drive_controller 控制器
    action_load_diff_drive_controller = launch.actions.ExecuteProcess(
    # 指令的意义是加载并激活名为 fishbot_diff_drive_controller 的控制器
    cmd=[
        'ros2', 'control', 'load_controller',
        'fishbot_diff_drive_controller',
        '--set-state', 'active'
    ],
    output='screen'
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
        action_launch_gazebo,
        action_joint_state_publisher,
        action_spawn_entity,
        # action_rviz_node,
        #RegisterEventHandler用于注册事件处理器,当action_spawn_entity进程退出时,触发加载joint_state_controller
        launch.actions.RegisterEventHandler(
            #OnProcessExit事件处理器,当指定进程退出时触发
            event_handler=launch.event_handlers.OnProcessExit(
                #指定要监视的进程
                target_action=action_spawn_entity,
                #指定进程退出时要执行的操作
                on_exit=[action_load_joint_state_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
            #OnProcessExit事件处理器,当指定进程退出时触发
            event_handler=launch.event_handlers.OnProcessExit(
                #指定要监视的进程
                target_action=action_load_joint_state_controller,
                #指定进程退出时要执行的操作
                on_exit=[action_load_diff_drive_controller],
            )
        ),
    ])