import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'puzzlebot_pallet_loader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['best.pt']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.npz')),
        # Modelos HMM de reconocimiento de voz
        (os.path.join('lib', 'python3.10', 'site-packages', package_name, 'models'),
            glob(package_name + '/models/*.pkl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='harumkim09@gmail.com',
    description='Puzzlebot Pallet Loader Final Project',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_publisher = puzzlebot_pallet_loader.camera_publisher:main',
            'voice_recognition = puzzlebot_pallet_loader.voice_recognition:main',
            'detector_node = puzzlebot_pallet_loader.detector_node:main',
            'hybrid_a_star_bug0_node = puzzlebot_pallet_loader.hybrid_a_star_bug0_node:main',
            'odometry_node = puzzlebot_pallet_loader.odometry_node:main',
            'ekf_aruco_localization_node = puzzlebot_pallet_loader.ekf_aruco_localization_node:main',
            'aruco_detector_node = puzzlebot_pallet_loader.aruco_detector_node:main',
            'fsm_control_node = puzzlebot_pallet_loader.fsm_control_node:main',
            'qr_align = puzzlebot_pallet_loader.qr_align:main',
            'mcl_localization_node = puzzlebot_pallet_loader.mcl_localization_node:main',
        ],
    },
)
