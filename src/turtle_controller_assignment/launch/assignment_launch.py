from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for Assignment Tasks 3-7
    
    Starts:
    - turtlesim simulator
    - all required service and action servers
    - auto turtle spawner (Task 3)
    - action client (Task 6) after delay
    """
    
    # Task 7: Start turtlesim simulator
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )
    
    # Supporting services
    turtle_name_manager = Node(
        package='turtle_controller_assignment',
        executable='turtle_name_manager',
        name='turtle_name_manager',
        output='screen'
    )
    
    turtle_monitor_service = Node(
        package='turtle_controller_assignment',
        executable='turtle_monitor_service',
        name='turtle_monitor_service',
        output='screen'
    )
    
    # Task 3: Auto spawner
    auto_turtle_spawner = Node(
        package='turtle_controller_assignment',
        executable='auto_turtle_spawner',
        name='auto_turtle_spawner',
        output='screen',
        parameters=[{'spawn_interval': LaunchConfiguration('spawn_interval')}]
    )
    
    # Task 4: Closest turtle service server
    closest_turtle_service = Node(
        package='turtle_controller_assignment',
        executable='closest_turtle_service',
        name='closest_turtle_service',
        output='screen'
    )
    
    # Task 5: Collection action server
    turtle_collection_server = Node(
        package='turtle_controller_assignment',
        executable='turtle_collection_server',
        name='turtle_collection_server',
        output='screen'
    )
    
    # Task 6: Collection action client
    turtle_collection_client = Node(
        package='turtle_controller_assignment',
        executable='turtle_collection_client',
        name='turtle_collection_client',
        output='screen'
    )
    
    name_manager_on_turtlesim_start = RegisterEventHandler(
        OnProcessStart(
            target_action=turtlesim_node,
            on_start=[turtle_name_manager]
        )
    )

    monitor_on_turtlesim_start = RegisterEventHandler(
        OnProcessStart(
            target_action=turtlesim_node,
            on_start=[turtle_monitor_service]
        )
    )

    closest_on_monitor_start = RegisterEventHandler(
        OnProcessStart(
            target_action=turtle_monitor_service,
            on_start=[closest_turtle_service]
        )
    )

    server_on_closest_start = RegisterEventHandler(
        OnProcessStart(
            target_action=closest_turtle_service,
            on_start=[turtle_collection_server]
        )
    )

    spawner_on_server_start = RegisterEventHandler(
        OnProcessStart(
            target_action=turtle_collection_server,
            on_start=[auto_turtle_spawner]
        )
    )

    client_on_server_start = RegisterEventHandler(
        OnProcessStart(
            target_action=turtle_collection_server,
            on_start=[turtle_collection_client]
        )
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('spawn_interval', default_value='5.0'),
        turtlesim_node,
        name_manager_on_turtlesim_start,
        monitor_on_turtlesim_start,
        closest_on_monitor_start,
        server_on_closest_start,
        spawner_on_server_start,
        client_on_server_start
    ])
