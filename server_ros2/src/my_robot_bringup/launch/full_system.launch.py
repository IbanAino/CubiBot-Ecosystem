import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_description = get_package_share_directory('my_robot_description')
    pkg_bringup = get_package_share_directory('my_robot_bringup')

    urdf_path = os.path.join(pkg_description, 'urdf', 'simple_robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    rviz_config = os.path.join(pkg_bringup, 'rviz', 'robot_view.rviz')

    # --- Argument de launch : choix du type de connexion ---
    connection_type_arg = DeclareLaunchArgument(
        'connection_type',
        default_value='serial',
        description="Type de connexion au robot : 'serial' ou 'wifi'"
    )
    connection_type = LaunchConfiguration('connection_type')

    # --- Nœud Description ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
            'publish_frequency': 30.0
        }]
    )

    # --- État des articulations ---
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'rate': 30   
        }]
    )

    # --- Nœud Driver : variante série (active si connection_type == 'serial') ---
    driver_serial = Node(
        package='my_robot_driver',
        executable='serial_bridge',
        output='screen',
        parameters=[{'port': '/dev/ttyACM0', 'baudrate': 9600}],
        condition=IfCondition(EqualsSubstitution(connection_type, 'serial')),
    )

    # --- Nœud Driver : variante WiFi (active si connection_type == 'wifi') ---
    driver_wifi_odom = Node(
        package='my_robot_driver',
        executable='wifi_bridge_odom',
        output='screen',
        parameters=[{'host': '0.0.0.0', 'port': 5007}],
        condition=IfCondition(EqualsSubstitution(connection_type, 'wifi')),
    )

    # --- Nœud Wifi Bridge Lidar ---
    driver_wifi_lidar = Node(
        package='my_robot_driver',
        executable='wifi_bridge_lidar',
        output='screen',
        parameters=[{
            'host': '0.0.0.0', 
            'port': 5006,
            'use_sim_time': False # En phase avec le reste du robot
        }],
        condition=IfCondition(EqualsSubstitution(connection_type, 'wifi')),
    )
    
    # --- Nœud du fake Lidar ---
    driver_fake_lidar = Node(
        package='my_robot_driver',
        executable='fake_lidar',
        output='screen',
    )
    
    # --- Nœud time syncer ---
    time_syncer = Node(
        package='my_robot_driver',
        executable='time_syncer',
        output='screen',
    )
    
    # --- Nœud Commandes ---
    teleop_gui = Node(
        package='my_robot_commands',
        executable='teleop_gui',
        output='screen',
    )

    # --- Nœud RViz ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # --- Nœud Wifi Commands ---
    driver_wifi_cmd = Node(
        package='my_robot_driver',
        executable='wifi_bridge_cmd',
        output='screen',
        parameters=[{
            'robot_ip': '192.168.1.11', 
            'port': 5010,
        }]
    )

    return LaunchDescription([
        connection_type_arg,
        robot_state_publisher,
        joint_state_publisher,
        driver_serial,
        driver_wifi_odom,
        driver_wifi_lidar,
        #driver_fake_lidar,
        teleop_gui,
        driver_wifi_cmd,
        rviz,
        time_syncer,
    ])
