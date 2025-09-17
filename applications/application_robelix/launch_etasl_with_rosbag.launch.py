import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
import datetime


def generate_launch_description():
    # Get paths
    # simulation_arg = DeclareLaunchArgument(
    #     'simulation',
    #     description='Set to "true" for simulation mode, "false" for real hardware'
    # )
    # simulation = LaunchConfiguration('simulation')




    your_package_dir = get_package_share_directory('board_localization')
    rviz_config_path = os.path.join(your_package_dir, 'rviz', 'setup.rviz')

    realsense_pkg_dir = get_package_share_directory('realsense2_camera')
    realsense_launch_path = os.path.join(realsense_pkg_dir, 'launch', 'rs_launch.py')

    urdf_file_name = 'robot_models/urdf_models/robot_setups/neura_maira7M/use_case_setup_neura_maira7M.urdf'
    config_rviz_file = 'robot_models/urdf_models/robot_setups/neura_maira7M/rviz_config.rviz' # Replace with your RViz config file path

    # urdf_file = os.path.join( get_package_share_directory('mav_description'), urdf_file_name)
    urdf_file = os.path.join( get_package_share_directory('etasl_ros2_application_template'), urdf_file_name)
    rviz_file = os.path.join( get_package_share_directory('etasl_ros2_application_template'), config_rviz_file)
    # config_file_path = os.path.join( get_package_share_directory('etasl_ros2_application_template'), "applications", "application_robelix", "application_robelix_fixed.setup.json")
    config_file_path = "$[etasl_ros2_application_template]/applications/application_robelix/application_robelix_fixed.setup.json"

    timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    file_bag = f'bag_{timestamp}'
    bag_output_path = os.path.join( get_package_share_directory('etasl_ros2_application_template'), "rosbags", file_bag)


    print("THE URDF NAME IS: " + urdf_file)
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

        
    return LaunchDescription([
        # simulation_arg,

        Node(
            package='etasl_ros2',
            executable='etasl_node',
            name='etasl_node',
            output='screen',
            parameters=[
                {'config_file': config_file_path},
                {'simulation': False}
            ]
        ),
        launch.actions.ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', bag_output_path,
                '/joint_states'
            ],
            output='log'
        )
    ])
