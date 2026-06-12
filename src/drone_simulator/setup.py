import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'drone_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Источник истины по конфигам — configs/ в корне (§7). Сюда ставим копию,
        # чтобы launch находил её по стабильному share-пути. Правка configs/ требует
        # пересборки, либо передать bridge_config:=<путь> напрямую (см. sim.launch.py).
        (os.path.join('share', package_name, 'config'),
         glob('../../configs/simulator/camera_bridge.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem',
    maintainer_email='artemlukyanovich@gmail.com',
    description='Запуск SITL, мосты, описание мира/дрона. Фаза 1 — пустой каркас, наполняется в Фазе 2.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # Нод пока нет — появятся в Фазе 2 (мосты, запуск SITL).
        ],
    },
)
