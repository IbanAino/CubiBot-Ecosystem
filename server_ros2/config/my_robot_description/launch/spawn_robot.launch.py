import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'simple_robot.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # Démarrage de Gazebo avec un monde vide
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': 'empty.sdf'}.items(),
    )

    # Publie le TF et le robot_description à partir de l'URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Fait apparaître le robot dans Gazebo à partir du topic robot_description
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'simple_robot',
            '-z', '0.1'
        ],
        output='screen',
    )

    # Pont ROS2 <-> Gazebo pour la commande de vitesse
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
    ])
