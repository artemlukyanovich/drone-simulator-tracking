from setuptools import find_packages, setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem',
    maintainer_email='artemlukyanovich@gmail.com',
    description='Управление дроном: follower_node (offboard-цикл, P/PID). Фаза 1 — заглушка.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'follower_node = drone_control.follower_node:main',
        ],
    },
)
