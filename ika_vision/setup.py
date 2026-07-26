from setuptools import find_packages, setup

package_name = 'ika_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emre',
    maintainer_email='emreerenatalay66@gmail.com',
    description='RAGNAR IKA - Hailo YOLO goruntu isleme node\'u (Gorev 4)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = ika_vision.yolo_node:main',
            'mock_yolo_cpu = ika_vision.mock_yolo_node_cpu:main',
        ],
    },
)
