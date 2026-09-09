import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction  # <--- 1. BU IMPORT'U EKLE

def generate_launch_description():
    pkg_name = 'rover_bringup'
    
    # Dosya yollarını bul
    urdf_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'rover.urdf')
    ekf_filter_node_odom_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'ekf_filter_node_odom.yaml')
    ekf_filter_node_map_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'ekf_filter_node_map.yaml')
    navsat_transform_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'navsat_transform.yaml')
    twist_mux_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'twist_mux.yaml')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()


    return LaunchDescription([
        # 1. Robot Modelini Yayınla (URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_config]
        ),
        Node(
            package='rover_bringup',
            executable='empty_map_pub',
            name='empty_map_publisher',
            output='screen'
        ),
        Node(
            package='rover_bringup',
            executable='set_datum_auto',
            name='set_datum_auto',
            output='screen'
        ),
        Node(
            package='bno055',
            executable='bno055',
            name='bno055',
            output='screen',
            parameters=[{
                'connection_type': 'uart',
                'uart_port': '/dev/ttyAMA0',
                'uart_baudrate': 115200,
                'frame_id': 'imu_link',
                'data_query_frequency': 35
            }]
        ),
        # Hesai XT16 Lidar (eth0, UDP 2368; config: install/hesai_ros_driver/share config.yaml)
        # output='log': surucu her frame'de stdout'a "raw frame:... points:64000" printf'i
        # basiyor (ROS logger degil, susturulamaz). 'log' ile bu terminale degil
        # ~/.ros/log dosyasina gider; terminal temiz kalir.
        Node(
            namespace='hesai_ros_driver',
            package='hesai_ros_driver',
            executable='hesai_ros_driver_node',
            name='hesai_ros_driver_node',
            output='log'
        ),

        # 5. SWEGEO RTK GPS Driver
       Node(
            package='swegeo_driver',
            executable='swegeo_node',
            name='swegeo_gps_driver',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200,
                'message_type': 'ascii', # binary or ascii
                'ntrip_enable': True, # True or False
                # NTRIP caster + account are supplied through the environment.
                # Copy ntrip.env.example to ntrip.env, fill it in, and source it
                # before launching. ntrip.env is git-ignored.
                'ntrip_host':       os.environ.get('NTRIP_HOST', ''),
                'ntrip_port':   int(os.environ.get('NTRIP_PORT', '80')),
                'ntrip_mountpoint': os.environ.get('NTRIP_MOUNTPOINT', 'MSM_GNSS'),
                'ntrip_user':       os.environ.get('NTRIP_USERNAME', ''),
                'ntrip_pass':       os.environ.get('NTRIP_PASSWORD', ''),
            }]
        ),


        # ---------------------------------------------
        # 3. LOKAL EKF 
        # ---------------------------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_filter_node_odom_config],
            remappings=[
                ('odometry/filtered', '/odometry/local')
            ]
        ),
        
        Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_transform_config,
                    {'yaw_offset': 0.0}], 
        respawn=True,
        respawn_delay=4.0,
        remappings=[
            ('imu', '/gps/imu'),
            ('gps/fix', '/fix'),
            ('odometry/filtered', '/odometry/local')
        ],
        arguments=['--ros-args', '--log-level', 'warn']
        ),  

        # ---------------------------------------------
        # 4. GLOBAL EKF
        # ---------------------------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_filter_node_map_config],
            remappings=[
                ('odometry/filtered', '/odometry/global')
            ]
        )
    ])