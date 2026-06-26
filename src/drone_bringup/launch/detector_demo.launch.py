"""detector_demo — запуск только detector_node с конфигом (Фаза 3, инкремент 2).

Поднимает детектор поверх уже работающего sim (камера /camera/image должна идти —
см. sim.launch.py). Нужен для изолированной проверки перцепции до контроллера:
смотрим /perception/target и /perception/image. Полный пайплайн (детектор+контроллер)
соберётся в tracking_demo.launch.py позже (инкремент 4).

Конфиг ищется по приоритету:
  1) аргумент config:=<абсолютный путь>;
  2) источник истины configs/perception/detector.yaml (если запуск из корня проекта);
  3) копия в share пакета drone_perception.
Параметр model_path в конфиге относителен корня проекта — запускать из корня.
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
        repo_cfg = os.path.abspath('configs/perception/detector.yaml')
        share_cfg = os.path.join(
            get_package_share_directory('drone_perception'), 'config', 'detector.yaml')
        config = repo_cfg if os.path.isfile(repo_cfg) else share_cfg
    return [Node(
        package='drone_perception',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[config],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value='',
            description='Абсолютный путь к yaml детектора (пусто → configs/perception/detector.yaml).'),
        OpaqueFunction(function=_resolve_config),
    ])
