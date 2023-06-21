from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    path2_pkg = 'ucsd_robocar_path2_pkg'
    config_file = 'gps_path_provider.yaml'
    gps_node_name = 'gps_path_provider_node'
    error_node_name = 'gps_error_node'
    diagnostic_node_name = 'diagnostics_node'
    imu_node_name = ''
    imutransformer_node_name = 'imu_transformer_node'
    gpstransformer_node_name = 'gps_transformer_node'
    

    ekf_config_file = os.path.join(
        get_package_share_directory(path2_pkg),
        'config',
        'ekf.yaml')

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory(path2_pkg),
        'config',
        config_file)


    path_node = Node(
        package=path2_pkg,
        executable=gps_node_name,
        output='screen',
        parameters=[config])

    error_node = Node(
        package=path2_pkg,
        executable=error_node_name,
        output='screen',
        parameters=[config])

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config_file])

    """
    diagnostics_node = Node(
        package=path2_pkg,
        executable=diagnostic_node_name,
        output='screen',
        parameters=[config])
    """

    """
    imutransform_node = Node(
        package=path2_pkg,
        executable=imutransformer_node_name,
        output='screen')
    """
    """
    gpstransform_node = Node(
        package=path2_pkg,
        executable=gpstransformer_node_name,
        output='screen',
        parameters=[config])
        """
    
    ##### 6/11 UPDATE - EKF
      # Start the navsat transform node which converts GPS data into the world coordinate frame
    start_navsat_transform_cmd = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[('imu', '/imu_topic'),
                    ('gps/fix', '/fix'), 
                    ('odometry/filtered', 'odometry/filtered')])

    # Start robot localization using an Extended Kalman filter...map->odom transform
    """
    start_robot_localization_global_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[('odometry/filtered', '/odom_combined')])
        """

    # Start robot localization using an Extended Kalman filter...odom->base_footprint transform
    """
    start_robot_localization_local_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[('odometry/filtered', '/odom_combined')])
        """

    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.00375', '-0.1375', '0.0', '0.0', '0.0', '0.0', '1.0', 'base_link', 'base_imu_link']
        )

    tf_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.00375', '-0.3725', '0.0', '0.0', '0.0', '0.0', '1.0', 'base_link', 'gps']
        )


    ld.add_action(path_node)
    ld.add_action(error_node)
    ld.add_action(start_robot_localization_cmd)
    #ld.add_action(diagnostics_node)
    
    #ld.add_action(imutransform_node)
    #ld.add_action(gpstransform_node)

    # 6/11 - EKF
    ld.add_action(start_navsat_transform_cmd)
    #ld.add_action(start_robot_localization_global_cmd)
    #ld.add_action(start_robot_localization_local_cmd)
    ld.add_action(tf_imu)
    ld.add_action(tf_gps)


    return ld