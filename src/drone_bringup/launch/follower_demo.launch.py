"""follower_demo — запуск только follower_node с конфигом (Фаза 3, инкремент 3).

Поднимает контроллер поверх работающего sim + uXRCE-DDS агента (sim.launch.py),
БЕЗ детектора — для изолированной проверки offboard-цикла на ФЕЙКОВОЙ цели:

  ros2 topic pub -r 10 /perception/target drone_interfaces/msg/Target \
    '{detected: true, offset_x: 0.4, offset_y: 0.0, area_ratio: 0.05}'

Ожидаемо: дрон армится, входит в offboard, взлетает на target_altitude и реагирует на
offset_x (доворот) и area_ratio (вперёд/назад). Полный пайплайн (детектор+контроллер)
соберётся в tracking_demo.launch.py (инкремент 4).

Конфиг: аргумент config:=<путь>, иначе configs/control/follower.yaml (из корня проекта),
иначе копия в share пакета drone_control.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_config(context):
    config = LaunchConfiguration('config').perform(context)
    if not config:
        repo_cfg = os.path.abspath('configs/control/follower.yaml')
        share_cfg = os.path.join(
            get_package_share_directory('drone_control'), 'config', 'follower.yaml')
        config = repo_cfg if os.path.isfile(repo_cfg) else share_cfg
    return [Node(
        package='drone_control',
        executable='follower_node',
        name='follower_node',
        output='screen',
        parameters=[config],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value='',
            description='Абсолютный путь к yaml контроллера (пусто → configs/control/follower.yaml).'),
        OpaqueFunction(function=_resolve_config),
    ])
