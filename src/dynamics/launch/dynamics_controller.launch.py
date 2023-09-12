import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription, launch_description_sources
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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

    bridge_dir = get_package_share_directory('rosbridge_server')
    bridge_launch =  IncludeLaunchDescription(launch_description_sources.FrontendLaunchDescriptionSource(bridge_dir + '/launch/rosbridge_websocket_launch.xml')) 
    

    sys_shut_down = RegisterEventHandler(OnProcessExit(
	        target_action=dynamics,
    on_exit=[
                LogInfo(msg=(f'The Scenario has ended!')),
                EmitEvent(event=Shutdown(
                    reason='Finished'))
		            ]		
	    ))

    
    ld.add_action(bridge_launch)
    ld.add_action(dynamics)
    ld.add_action(controller)
    ld.add_action(sys_shut_down)

    return ld