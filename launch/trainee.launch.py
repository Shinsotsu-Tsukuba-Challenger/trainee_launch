import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    declared_arguments = []

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("trainee_description"), "urdf", "trainee.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("trainee_launch"),
            "config", "param",
            "trainee_controllers.yaml",
        ]
    )

    speak_list = os.path.join(
        get_package_share_directory('trainee_speak'), 'config', 'speak_list.param.yaml')
    voice_config = os.path.join(
        get_package_share_directory('trainee_speak'), 'config', 'voice_config.param.yaml')

    speak_node  = Node(
        name='trainee_speak',
        package='trainee_speak',
        executable='trainee_speak_node',
        arguments=[speak_list],
        parameters=[voice_config],
        output='screen'
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            robot_controllers
        ],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/trainee/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["trainee", "--controller-manager", "/controller_manager"],
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    exec_speak = ExecuteProcess(
        cmd=[
                'ros2', 'topic', 'pub', '--once', '/speak', 'std_msgs/String',
                "data: 'トレーニーが起動しました'"
        ],
        output='screen'
    )

    nodes = [
        speak_node,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        exec_speak,
    ]

    return LaunchDescription(declared_arguments + nodes)
