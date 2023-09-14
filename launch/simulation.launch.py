from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    descr_path = get_package_share_path('dexhand_description')
    default_model_path = descr_path / 'urdf/dexhand.xacro'
    rviz_path = descr_path / 'rviz/urdf.rviz'

    urdf = ParameterValue(Command(['xacro ', str(default_model_path)]),
                                     value_type=str)

    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf}]),
        Node(
            package='dexhand_gesture_controller',
            executable='gesture_controller',
            name='gesture_controller',
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(rviz_path)]),
        
        
    ])