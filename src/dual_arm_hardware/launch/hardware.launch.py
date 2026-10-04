"""Real hardware only; never starts a synthetic Joint State Publisher."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def setup(context):
    description = Path(get_package_share_directory('dual_arm_description'))
    xml = xacro.process_file(str(description/'urdf/dual_arm.urdf.xacro')).toxml()
    driver = Node(package='dual_arm_hardware', executable='hardware_node.py', output='screen',
                  parameters=[{'hardware_config': LaunchConfiguration('hardware_config').perform(context),
                               'description_config': str(description/'config')}])
    return [
        RegisterEventHandler(OnProcessExit(target_action=driver,
            on_exit=[EmitEvent(event=Shutdown(reason='Hardware driver exited; stopping the hardware launch'))])),
        driver,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': xml}], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(description/'rviz/dual_arm.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('dual_arm_hardware'))
    return LaunchDescription([
        DeclareLaunchArgument('hardware_config', default_value=str(share/'config/hardware.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=setup),
    ])
