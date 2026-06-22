from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    use_rviz = LaunchConfiguration('rviz')
    voxeloctree_file = LaunchConfiguration('voxeloctree_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    use_pose_pub = LaunchConfiguration('use_pose_pub')

    ld.add_action(DeclareLaunchArgument('rviz', default_value='True'))
    ld.add_action(DeclareLaunchArgument('voxeloctree_file', default_value='/home/abc/map2.voxeloctree'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('x', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('y', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('z', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('roll', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('pitch', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('use_pose_pub', default_value='True'))

    livo_params_file = os.path.join(get_package_share_directory('fast_livo'), 'config', 'avia.yaml')
    camera_params_file = os.path.join(get_package_share_directory('fast_livo'), 'config', 'camera_pinhole.yaml')
    rviz_config_file = os.path.join(get_package_share_directory('fast_livo'), 'rviz_cfg', 'fast_livo2.rviz')

    # Static TF
    static_lidar_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=['0.04165', '0.02326', '-0.0284', '0.0', '0.0', '0.0', 'camera_init', 'aft_mapped']
    )
    ld.add_action(static_lidar_tf_pub)

    # Fast LIVO node
    livo = Node(
        package='fast_livo',
        executable='fastlivo_mapping',
        name='laserMapping',
        parameters=[
            livo_params_file,
            camera_params_file,
            {
                'voxeloctree_file_path': voxeloctree_file,
                'use_sim_time': use_sim_time
            }
        ],
        output='screen'
    )
    ld.add_action(livo)

    # RViz2 Node
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    ld.add_action(rviz_cmd)

# Initial Pose Publisher Node (only runs if use_pose_pub is True)
    initial_pose_pub = Node(
        condition=IfCondition(use_pose_pub),
        package='initial_pose_publisher',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        parameters=[{
            'map_frame': 'camera_init',
            
            # Your exact 120-second global transformation matrix
            # 'localization_matrix': [
            #     -0.530889,  0.596476, -0.601974, -2.099868,
            #     -0.492068, -0.795299, -0.354075, -12.561502,
            #     -0.689947,  0.108238,  0.715723, -0.465683,
            #      0.0,       0.0,       0.0,       1.0
            # ],
            #119.9
            # 'localization_matrix': [
            #     -0.285499,  0.918926, -0.272164, -2.255120,
            #     -0.911477, -0.334005, -0.239330, -12.991700,
            #     -0.310817,  0.179739,  0.933333, -0.538312,
            #      0.0,       0.0,       0.0,       1.0],

            #at origin(0,0,0)
            'localization_matrix':[ 1.0,  0.0,  0.0,  0.0,
                                   0.0,  1.0,  0.0,  0.0,
                                   0.0,  0.0,  1.0,  0.0,
                                   0.0,  0.0,  0.0,  1.0],

            'x': x,
            'y': y,
            'z': z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    ld.add_action(initial_pose_pub)
    return ld
