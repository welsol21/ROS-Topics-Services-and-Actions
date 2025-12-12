from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # World name (Gazebo)
    world_name = LaunchConfiguration("world_name")

    declared_arguments = [
        DeclareLaunchArgument(
            "world_name",
            default_value="empty",
            description="Gazebo world name",
        ),
    ]

    pkg_share = FindPackageShare("ur3_description")
    # Robot description from xacro
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "ur3.urdf.xacro"])
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            urdf_file,
        ]
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Gazebo (ros_gz_sim)
    gz_sim = Node(
        package="ros_gz_sim",
        executable="gz_sim",
        arguments=["-r", "-v", "4", f"{world_name}.sdf"],
        output="screen",
    )

    # Bridge for /clock and /world/empty/set_pose service
    gz_bridge_params = PathJoinSubstitution(
        [pkg_share, "config", "gazebo_bridge.yaml"]
    )
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": gz_bridge_params}],
    )

    # Spawn table
    table_sdf = PathJoinSubstitution(
        [pkg_share, "models", "table.sdf"]
    )
    spawn_table = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "table",
            "-file", table_sdf,
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
        output="screen",
    )

    # Spawn cube
    cube_sdf = PathJoinSubstitution(
        [pkg_share, "models", "cube.sdf"]
    )
    spawn_cube = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "cube",
            "-file", cube_sdf,
            "-x", "0.0",
            "-y", "0.5",
            "-z", "1.0",
        ],
        output="screen",
    )

    # Spawn UR3 robot at (0,0,1) using robot_description topic
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "ur3",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "1.0",
        ],
        output="screen",
    )

    # Spawners for controllers (joint state broadcaster + forward position controller)
    jsb_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    forward_pos_spawner = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "forward_position_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    ld = LaunchDescription(declared_arguments)
    ld.add_action(gz_sim)
    ld.add_action(gazebo_bridge)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_table)
    ld.add_action(spawn_cube)
    ld.add_action(spawn_robot)
    ld.add_action(jsb_spawner)
    ld.add_action(forward_pos_spawner)

    return ld
