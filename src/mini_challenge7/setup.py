from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'mini_challenge7'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harum Kim',
    maintainer_email='harumkim09@gmail.com',
    description='EKF + ArUco navigation challenge',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_detector_node = mini_challenge7.aruco_detector_node:main',
            'ekf_aruco_localization_node = mini_challenge7.ekf_aruco_localization_node:main',
            'covariance_visualizer_node = mini_challenge7.covariance_visualizer_node:main',
            'bug0_navigation_node = mini_challenge7.bug0_navigation_node:main',
            'bug2_navigation_node = mini_challenge7.bug2_navigation_node:main',
            'bug1_navigation_node = mini_challenge7.bug1_navigation_node:main',
            'tangent_bug_node = mini_challenge7.tangent_bug_node:main',
        ],
    },
)
