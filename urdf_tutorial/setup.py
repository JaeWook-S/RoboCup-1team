import os
from glob import glob
from setuptools import setup

package_name = 'urdf_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), # 두 줄 추가 -> 두 폴더가 컴파일에 포함될 수 있도록 
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaewook',
    maintainer_email='jaewook@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_image_logger = urdf_tutorial.camera_image_logger:main',
            'odometry_logger = urdf_tutorial.odometry_logger:main',
        ],
    },
)
