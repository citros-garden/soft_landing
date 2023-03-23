import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
def generate_launch_description():
    ld = LaunchDescription()
    dynamics_config = os.path.join(
        get_package_share_directory('dynamics'),
        'config',
        'params.yaml'
        )
    controller_config = os.path.join(
        get_package_share_directory('controller'),
        'config',
        'params.yaml'
        )
    dynamics=Node(
        package = 'dynamics',
        name = 'dynamics',
        executable = 'dynamics',
        parameters = [dynamics_config]
    )
    controller=Node(
        package = 'controller',
        name = 'controller',
        executable = 'controller',
        parameters = [controller_config]
    )

    free_dynamics=Node(
        package = 'dynamics',
        name = 'free_dynamics',
        executable = 'dynamics',
        parameters = [{'r_x0': 0.0, 'r_y0': 0.0, 'r_z0': 0.0, 'v_x0': 0.0, 'v_y0': 0.0, 'v_z0': 0.0 , 'g_x':0.0 ,'g_y':0.0 ,'g_z':0.0}],
        remappings=[
            ('dynamics/position', '/free_dynamics/position'),
            ('dynamics/velocity', '/free_dynamics/velocity'),
            ('controller/command', '/free_controller/command'),
        ]
    )
    free_controller=Node(
        package = 'controller',
        name = 'free_controller',
        executable = 'controller',
        parameters = [{'setpoint_r_x': 0.0, 'setpoint_r_y': 0.0, 'setpoint_r_z': 10.0 , 'setpoint_v_x': 0.0 , 'setpoint_v_y': 0.0,'setpoint_v_z': 0.0}],
        remappings=[
            ('dynamics/position', '/free_dynamics/position'),
            ('dynamics/velocity', '/free_dynamics/velocity'),
            ('controller/command', '/free_controller/command'),
        ]
        
    )
    bridge_dir = get_package_share_directory('rosbridge_server')
    bridge_launch =  IncludeLaunchDescription(launch_description_sources.FrontendLaunchDescriptionSource(bridge_dir + '/launch/rosbridge_websocket_launch.xml')) 
    ld.add_action(bridge_launch)
    ld.add_action(dynamics)
    ld.add_action(controller)
    # ld.add_action(free_dynamics)
    # ld.add_action(free_controller)
    return ld