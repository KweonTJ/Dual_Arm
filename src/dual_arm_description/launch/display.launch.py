"""Display only: publishes synthetic joint states and never connects to motors."""
from pathlib import Path
import os
import tempfile
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('dual_arm_description'))
    dims_path = LaunchConfiguration('dimensions_file').perform(context)
    mappings = {'dimensions_file': dims_path}
    for key in ('mount_spacing', 'mount_height'):
        value = LaunchConfiguration(key).perform(context)
        if value:
            mappings[key] = value
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    description = Path(urdf_file).read_text() if urdf_file else xacro.process_file(
        str(share / 'urdf/dual_arm.urdf.xacro'), mappings=mappings).toxml()
    pose = yaml.safe_load((share / 'config/preview_pose.yaml').read_text())['joint_state_publisher']['ros__parameters']
    # Humble JSP accepts a description file, not a robot_description parameter.
    # Pin GUI and non-GUI inputs to the exact URDF used by the guard and RViz.
    fd, preview_description = tempfile.mkstemp(prefix='dual_arm_preview_', suffix='.urdf')
    with os.fdopen(fd, 'w') as stream:
        stream.write(description)

    def cleanup(_context):
        Path(preview_description).unlink(missing_ok=True)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             name='joint_state_publisher', arguments=[preview_description], parameters=[pose],
             remappings=[('joint_states', 'preview/joint_targets')], condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', arguments=[preview_description], parameters=[pose],
             remappings=[('joint_states', 'preview/joint_targets')], condition=UnlessCondition(LaunchConfiguration('gui'))),
        Node(package='dual_arm_description', executable='limited_joint_state_publisher.py',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz/dual_arm.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('dual_arm_description'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('urdf_file', default_value='',
                              description='Pre-expanded URDF; overrides Xacro dimensions when set'),
        DeclareLaunchArgument('dimensions_file', default_value=str(share / 'config/dimensions.yaml')),
        DeclareLaunchArgument('mount_spacing', default_value='', description='Override shoulder centre spacing in metres; default fits the crossbar ends'),
        DeclareLaunchArgument('mount_height', default_value='', description='Override shoulder shaft height in metres; crossbar moves with the arms'),
        OpaqueFunction(function=setup),
    ])
