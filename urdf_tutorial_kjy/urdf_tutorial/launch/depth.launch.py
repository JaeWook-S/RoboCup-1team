import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "urdf_tutorial"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "robot_6.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot"],
        output="screen",
    )

    teleop = Node(
    package='teleop_twist_keyboard',
    executable='teleop_twist_keyboard',
    name='teleop_keyboard',
    output='screen',
    prefix='xterm -e',  # 새 터미널 창에서 실행하여 키보드 입력 받게 함
    remappings=[
        ('/cmd_vel', '/cmd_vel')  # 필요 시 수정 가능
    ]
)

    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
            teleop,
        ]
    )