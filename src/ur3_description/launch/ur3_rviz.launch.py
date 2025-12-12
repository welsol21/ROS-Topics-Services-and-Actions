from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to URDF (xacro) and RViz config
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("ur3_description"), "urdf", "ur3.urdf.xacro"]
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ur3_description"), "rviz", "ur3.rviz"]
    )

    # xacro -> URDF string
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str,
    )

    # Publishes TF based on /joint_states and robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # GUI that publishes /joint_states — важно передать тот же robot_description
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            joint_state_publisher_gui,
            robot_state_publisher,
            rviz,
        ]
    )
