"""sim.launch.py — ROS2-сторона симулятора: мост PX4↔ROS2 (агент) + мост камеры.

PX4 SITL — отдельный процесс (make), поднимается scripts/run_px4_sitl.sh, НЕ здесь.
Этот launch поднимает то, что является ROS2-узлами/мостами (см. project_plan.md §8):

  1. Micro-XRCE-DDS-Agent — отдаёт uORB-топики PX4 в ROS2 (/fmu/*).
  2. ros_gz_bridge:parameter_bridge — перекладывает камеру Gazebo в /camera/image.

Демо Фазы 2 (два терминала):
    scripts/run_px4_sitl.sh
    ros2 launch drone_simulator sim.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drone_simulator')
    default_bridge_config = os.path.join(pkg_share, 'config', 'camera_bridge.yaml')

    xrce_port = LaunchConfiguration('xrce_port')
    bridge_config = LaunchConfiguration('bridge_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'xrce_port', default_value='8888',
            description='UDP-порт Micro-XRCE-DDS-Agent (по умолчанию совпадает с PX4 SITL).',
        ),
        DeclareLaunchArgument(
            'bridge_config', default_value=default_bridge_config,
            description=('YAML-конфиг ros_gz_bridge для камеры. По умолчанию — копия в share '
                         '(ставится при colcon build). Чтобы править без пересборки, передать '
                         'путь к configs/simulator/camera_bridge.yaml.'),
        ),

        # Мост PX4 ↔ ROS2. Это не ROS2-нода, а отдельный бинарь — запускаем как процесс.
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', xrce_port],
            name='micro_xrce_agent',
            output='screen',
        ),

        # Мост камеры Gazebo → ROS2 (/camera/image, /camera/camera_info).
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        ),
    ])
