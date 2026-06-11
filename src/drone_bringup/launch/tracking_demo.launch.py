"""tracking_demo — первый launch-файл пайплайна (Фаза 1).

Поднимает ноды-заглушки detector_node и follower_node, доказывая, что пакеты
собрались и оркестрируются через launch, а не хардкодом. По мере роста проекта сюда
добавятся мост ros_gz (камера), Micro-XRCE-DDS agent и параметры из configs/
(Фазы 2–3). drone_simulator пока без нод и здесь не участвует.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_perception',
            executable='detector_node',
            name='detector_node',
            output='screen',
        ),
        Node(
            package='drone_control',
            executable='follower_node',
            name='follower_node',
            output='screen',
        ),
    ])
