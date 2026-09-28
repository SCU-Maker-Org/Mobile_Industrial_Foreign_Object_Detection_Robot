from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('tracked_base')

    ekf_config  = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'slam.yaml')

    # ---------- launch 参数 ----------
    port_arg       = DeclareLaunchArgument('port',       default_value='/dev/motor')
    baud_arg       = DeclareLaunchArgument('baudrate',   default_value='230400')
    use_ekf_arg    = DeclareLaunchArgument('use_ekf',    default_value='true')
    use_slam_arg   = DeclareLaunchArgument('use_slam',   default_value='true')
    use_filter_arg = DeclareLaunchArgument('use_filter', default_value='true')
    crop_arg       = DeclareLaunchArgument(
        'crop_ranges_deg', default_value='[-45.0, 45.0]')

    # ---------- STM32 桥接 ----------
    bridge = Node(
        package='tracked_base',
        executable='stm32_bridge',
        name='stm32_bridge',
        output='screen',
        parameters=[{
            'port':       LaunchConfiguration('port'),
            'baudrate':   LaunchConfiguration('baudrate'),
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'imu_frame':  'imu_link',
            'publish_tf': False,
        }]
    )

    # ---------- 激光角度过滤 ----------
    scan_filter = Node(
        package='tracked_base',
        executable='scan_filter',
        name='scan_filter',
        output='screen',
        parameters=[{
            'input_topic':     '/scan',
            'output_topic':    '/scan_filtered',
            'crop_ranges_deg': LaunchConfiguration('crop_ranges_deg'),
        }],
        condition=IfCondition(LaunchConfiguration('use_filter')),
    )

    # ---------- EKF ----------
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        condition=IfCondition(LaunchConfiguration('use_ekf')),
    )

    # ---------- SLAM ----------
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
        condition=IfCondition(LaunchConfiguration('use_slam')),
    )

    # ---------- 静态 TF ----------
    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['--x', '0', '--y', '0', '--z', '0.05',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', 'imu_link'],
    )

    return LaunchDescription([
        port_arg, baud_arg, use_ekf_arg, use_slam_arg, use_filter_arg, crop_arg,
        bridge,
        scan_filter,
        ekf,
        slam,
        tf_imu,
    ])
