import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import time
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('main'),
        'config',
        'barelang.yaml'
        )

    motion_bridge_node=Node(
        package = 'motion_bridge',
        name = 'motion_bridge',
        executable = 'motion_bridge',
        parameters = [config],
        namespace = 'robot_3',
        output = 'screen'
    )

    main_strategy_node=Node(
        package = 'main',
        name = 'main_strategy',
        executable = 'main',
        parameters = [config],
        namespace = 'robot_3',
        emulate_tty=True,
        output = 'screen'
    )

    game_controller_node=Node(
        package = 'game_controller',
        name = 'game_controller',
        executable = 'game_controller',
        parameters = [config],
        namespace = 'robot_3',
        output = 'screen'
    )

    sensor_node=Node(
        package = 'sensor_oled',
        name = 'sensor',
        executable = 'sensor',
        parameters = [config],
        namespace = 'robot_3',
        output = 'screen'
    )

    receiver_node=Node(
        package = 'receiver',
        name = 'receiver',
        executable = 'receiver',
        parameters = [config],
        namespace = 'robot_3',
        output = 'screen'
    )

    oled_node=Node(
        package = 'sensor_oled',
        name = 'oled',
        executable = 'oled',
        parameters = [config],
        namespace = 'robot_3',
        output = 'screen'
    )

    localization_node=Node(
        package = 'localization',
        name = 'localization',
        executable = 'localization',
        parameters = [config],
        namespace = 'robot_3',
        emulate_tty=True,
        output = 'screen'
    )

    map_node=Node(
        package = 'localization',
        name = 'map',
        executable = 'map',
        parameters = [config],
        namespace = 'robot_3'
        # output = 'screen'
    )

    run_lua_node=Node(
        package = 'main',
        name = 'run_lua',
        executable = 'run_lua',
        parameters = [config],
        namespace = 'robot_3',
        #emulate_tty=True,
        output = 'screen'
    )

    path_finding_node=Node(
        package = 'path_finding',
        name = 'path_finding',
        executable = 'path_finding',
        parameters = [config],
        namespace = 'robot_3',
        #emulate_tty=True,
        output = 'screen'
    )

    ball_pose_node=Node(
        package = 'monitor_lokalisasi',
        name = 'monitor_lokalisasi',
        executable = 'monitor_lokalisasi',
        parameters = [config],
        namespace = 'robot_3',
        #emulate_tty=True,
        output = 'screen'
    )

    robot_coordination_node=Node(
        package = 'robot_coordination',
        name = 'robot_coordination',
        executable = 'robot_coordination',
        parameters = [config],
        namespace = 'robot_3',
        #emulate_tty=True,
        output = 'screen'
    )
    
    camera_odom_node=Node(
        package = 'new_localization',
        name = 'new_localization',
        executable = 'new_localization',
        parameters = [config],
        namespace = 'robot_3',
        #emulate_tty=True,
        output = 'screen'
    )

    ld.add_action(motion_bridge_node)
    ld.add_action(receiver_node)
    ld.add_action(sensor_node)
    ld.add_action(main_strategy_node)
    ld.add_action(game_controller_node)
    ld.add_action(path_finding_node)
    ld.add_action(robot_coordination_node)
    ld.add_action(oled_node)
    ld.add_action(ball_pose_node)
    ld.add_action(camera_odom_node)
    return ld
