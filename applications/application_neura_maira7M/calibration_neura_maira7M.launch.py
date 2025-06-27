import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from scipy.spatial.transform import Rotation as R



def generate_launch_description():

    # urdf_file_name = 'urdf/maira7M/urdf/maira7M.urdf'
    urdf_file_name = 'robot_models/urdf_models/robot_setups/neura_maira7M/use_case_setup_neura_maira7M.urdf'
    config_rviz_file = 'robot_models/urdf_models/robot_setups/neura_maira7M/rviz_config.rviz' # Replace with your RViz config file path

    # urdf_file = os.path.join( get_package_share_directory('mav_description'), urdf_file_name)
    urdf_file = os.path.join( get_package_share_directory('etasl_ros2_application_template'), urdf_file_name)
    rviz_file = os.path.join( get_package_share_directory('etasl_ros2_application_template'), config_rviz_file)

    print("THE URDF NAME IS: " + urdf_file)
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    
    r = R.from_euler('z', -45, degrees=True)
    rot_x, rot_y, rot_z, rot_w = r.as_quat()

    return LaunchDescription([
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': robot_desc}],
        #     arguments=[urdf_file]
        # ),
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
            arguments=['-d', rviz_file]  
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_pub_quat',
        #     arguments=[
        #         '0.2', '-0.2', '0.0',       # translation x y z
        #         str(rot_x), str(rot_y), str(rot_z), str(rot_w),# rotation quaternion x y z w
        #         'maira7M_root_link',               # parent frame
        #         'camera_link'             # child frame
        #     ],
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_pub_quat',
        #     arguments=[
        #         '0.0', '0.0', '0.0',       # translation x y z
        #         '0.0', '0.0', '0.0', '0.0',# rotation quaternion x y z w
        #         'maira7M_root_link',               # parent frame
        #         'camera_link'             # child frame
        #     ],
        # ),


        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_pub_quat2',
        #     arguments=[
        #         '-0.20', '-0.0', '-0.0',       # translation x y z
        #         '0.0', '0.0', '0.0', '1.0',# rotation quaternion x y z w
        #         'maira7M_flange',               # parent frame
        #         'marker_est_2'             # child frame
        #     ],
        # ),
    ])
