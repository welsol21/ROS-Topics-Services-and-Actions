from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
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

    # Gazebo Garden (ign-gazebo6) - compatible with ros-humble-gz-ros2-control
    gz_sim = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-v", "4", "empty.sdf"],
        output="screen",
        additional_env={"GZ_SIM_SYSTEM_PLUGIN_PATH": "/opt/ros/humble/lib"},
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

    # Spawn table via ign service command (Garden)
    spawn_table = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash", "-c",
                    "source /home/vlad/ros2_ws/install/setup.bash && "
                    "TABLE_SDF=$(ros2 pkg prefix ur3_description)/share/ur3_description/models/table.sdf && "
                    "ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 30000 "
                    "--req \"sdf_filename: \\\"$TABLE_SDF\\\", name: \\\"table\\\"\""
                ],
                output="screen",
            )
        ],
    )

    # Spawn cube via ign service command (Garden)
    spawn_cube = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash", "-c",
                    "source /home/vlad/ros2_ws/install/setup.bash && "
                    "CUBE_SDF=$(ros2 pkg prefix ur3_description)/share/ur3_description/models/cube.sdf && "
                    "ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 30000 "
                    "--req \"sdf_filename: \\\"$CUBE_SDF\\\", name: \\\"cube\\\", pose: {position: {y: 0.5, z: 1.0}}\""
                ],
                output="screen",
            )
        ],
    )

    # Spawn UR3 robot via robot_description topic (Garden)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash", "-c",
                    "source /home/vlad/ros2_ws/install/setup.bash && "
                    "ros2 topic echo --once /robot_description --field data | head -n -1 > /tmp/ur3_robot.urdf && "
                    "ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 30000 "
                    "--req \"sdf_filename: \\\"/tmp/ur3_robot.urdf\\\", name: \\\"ur3\\\", pose: {position: {z: 1.0}}\""
                ],
                output="screen",
            )
        ],
    )

    # Spawners for controllers (joint state broadcaster + forward position controller)
    jsb_spawner = TimerAction(
        period=8.0,
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
        period=10.0,
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
