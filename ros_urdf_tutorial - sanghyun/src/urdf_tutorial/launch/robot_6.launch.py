import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro  # xacro 모듈을 추가

def generate_launch_description():
    package_name = "urdf_tutorial"
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, "urdf", "robot_3.xacro")

    # xacro 파일을 처리하여 robot_description을 생성
    robot_description = xacro.process_file(xacro_file)
    robot_description_param = robot_description.toxml()

    # use_sim_time을 정의 (시뮬레이션 시간 사용 여부)
    use_sim_time = "true"

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), "launch", "robot_3.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Gazebo with specific world file
    world_path = os.path.join(
        get_package_share_directory(package_name), "config", "with_robot.world"
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_path}.items(),
    )

    # Spawn the robot entity in Gazebo (spawn_entity는 world가 로드된 후에 실행)
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "robot_1"],
        output="screen",
    )

    # Teleop node (example using teleop_twist_keyboard)
    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_keyboard",
        output="screen",
        prefix="xterm -e",  # Opens in new terminal, optional
    )

    # Launch Description 생성
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,  # Gazebo가 로드된 후 스폰 실행
        teleop_node,
    ])
