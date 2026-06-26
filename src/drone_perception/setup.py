import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'drone_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Копия конфига перцепции в share (источник истины — configs/perception/).
        (os.path.join('share', package_name, 'config'),
         glob('../../configs/perception/detector.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem',
    maintainer_email='artemlukyanovich@gmail.com',
    description='Перцепция дрона: detector_node (YOLO над /camera/image). Фаза 1 — заглушка.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detector_node = drone_perception.detector_node:main',
        ],
    },
)
