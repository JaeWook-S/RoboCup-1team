import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Launch argument
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Package paths
    urdf_pkg = get_package_share_directory("urdf_tutorial")
    gazebo_pkg = get_package_share_directory("gazebo_ros")

    # Process xacro file
    xacro_file = os.path.join(urdf_pkg, "urdf", "robot_3.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml(), "use_sim_time": use_sim_time}

    # Include robot_4 launch
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(urdf_pkg, "launch", "robot_4.launch.py")),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, "launch", "gazebo.launch.py"))
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot"],
        output="screen",
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true"
        ),
        rsp,
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])