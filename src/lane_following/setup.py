from glob import glob

from setuptools import setup

package_name = 'lane_following'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ddon',
    maintainer_email='caoquangddon5@gmail.com',
    description='Classical lane detection and pixel-gain steering preview.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_following_node = lane_following.lane_node:main',
            'lane_viewer = lane_following.debug_viewer:main',
            'lane_detect_image = lane_following.cli:main',
            'lane_synthetic_video = lane_following.synthetic:main',
            'lane_arm_console = lane_following.arm_console:main',
            'mock_vehicle = lane_following.mock_vehicle:main',
        ],
    },
)
