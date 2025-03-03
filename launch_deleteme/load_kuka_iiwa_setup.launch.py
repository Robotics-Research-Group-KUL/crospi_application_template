import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    urdf_file_name = 'robot_models/urdf_models/robot_setups/kuka_iiwa/use_case_setup_iiwa.urdf'
    config_rviz_file = 'robot_models/urdf_models/robot_setups/kuka_iiwa/rviz_config.rviz' # Replace with your RViz config file path

    urdf_file = os.path.join( get_package_share_directory('etasl_ros2_application_template'), urdf_file_name)
    rviz_file = os.path.join( get_package_share_directory('etasl_ros2_application_template'), config_rviz_file)

    print("THE URDF NAME IS: " + urdf_file)
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf_file]
        ),
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen'
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file]  # Replace with your RViz config file path
        )
    ])