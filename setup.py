from setuptools import setup
import os
from glob import glob

package_name = 'urdf_viewer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'resource/rviz'), glob('resource/rviz/*')),
        (os.path.join('share', package_name, 'urdf_viewer/urdf'), glob('urdf_viewer/urdf/*')),
        # モデルのファイルが適切にインストールされるように修正
        # (os.path.join('share', package_name, 'models'), glob('models/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='URDF Viewer Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = urdf_viewer.teleop_keyboard:main',
        ],
    },
)
