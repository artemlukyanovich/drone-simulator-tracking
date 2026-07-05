"""tracking_demo — полный пайплайн Фазы 3 «вижу → двигаюсь» (инкремент 4).

Поднимает РЕАЛЬНЫЙ пайплайн поверх работающего sim:
  follower_node (offboard P-цикл) + detector_node (YOLO → /perception/target),
каждый со своим конфигом из configs/. Это место, где перцепция начинает физически
управлять дроном (план §5, §8.10). Sim-сторона (PX4 + агент + камера-мост) поднимается
ОТДЕЛЬНО, до этого launch:

  Терминал 1: WORLD=src/drone_simulator/worlds/follow_target.sdf scripts/run_px4_sitl.sh
  Терминал 2: ros2 launch drone_simulator sim.launch.py           # агент + камера-мост
  Терминал 3: ros2 launch drone_bringup tracking_demo.launch.py    # ЭТОТ файл

Порядок старта нод намеренный (см. §9 — открытый блокер «полный пайплайн не армится»):
  • follower_node стартует в t=0 — успевает заармиться и взлететь, пока нет GPU-нагрузки;
  • detector_node стартует через detector_delay_s — тяжёлый YOLO-инференс приходит позже.
Это и митигация, и инструмент диагностики таймингового контеншна (lockstep). Ручки:
  detector_delay_s:=<сек>   задержка старта детектора (0.0 = одновременно, как в баге)
  detector_device:=cpu      оверрайд device детектора (проверить гипотезу контеншна GPU)
  detector_config:=<путь>   / follower_config:=<путь>   явные пути к yaml (иначе configs/).

Конфиги резолвятся: аргумент → configs/<...>.yaml (из корня проекта) → копия в share.
model_path в конфиге детектора относителен корня проекта — запускать из корня.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_config(arg_value, repo_rel, package):
    """Аргумент → configs/<repo_rel> (из корня проекта) → копия в share пакета."""
    if arg_value:
        return arg_value
    repo_cfg = os.path.abspath(repo_rel)
    if os.path.isfile(repo_cfg):
        return repo_cfg
    return os.path.join(get_package_share_directory(package), 'config', os.path.basename(repo_rel))


def _launch_setup(context):
    follower_cfg = _resolve_config(
        LaunchConfiguration('follower_config').perform(context),
        'configs/control/follower.yaml', 'drone_control')
    detector_cfg = _resolve_config(
        LaunchConfiguration('detector_config').perform(context),
        'configs/perception/detector.yaml', 'drone_perception')

    detector_delay = float(LaunchConfiguration('detector_delay_s').perform(context))
    detector_device = LaunchConfiguration('detector_device').perform(context)

    # device-оверрайд идёт ПОСЛЕ файла конфига — поздние параметры перекрывают ранние.
    detector_params = [detector_cfg]
    if detector_device:
        detector_params.append({'device': detector_device})

    follower = Node(
        package='drone_control',
        executable='follower_node',
        name='follower_node',
        output='screen',
        parameters=[follower_cfg],
    )
    detector = Node(
        package='drone_perception',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=detector_params,
    )

    # follower — сразу (армится/взлетает); detector — через задержку (см. docstring/§9).
    detector_action = (
        TimerAction(period=detector_delay, actions=[detector]) if detector_delay > 0.0
        else detector
    )
    return [follower, detector_action]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'follower_config', default_value='',
            description='Путь к yaml контроллера (пусто → configs/control/follower.yaml).'),
        DeclareLaunchArgument(
            'detector_config', default_value='',
            description='Путь к yaml детектора (пусто → configs/perception/detector.yaml).'),
        DeclareLaunchArgument(
            'detector_delay_s', default_value='0.0',
            description='Задержка старта detector_node, сек (даёт follower заармиться первым, §9).'),
        DeclareLaunchArgument(
            'detector_device', default_value='',
            description='Оверрайд device детектора: cuda|cpu (пусто → значение из конфига).'),
        OpaqueFunction(function=_launch_setup),
    ])
