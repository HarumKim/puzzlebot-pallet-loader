from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mini_challenge4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Valeria Meneses',
    maintainer_email='[EMAIL_ADDRESS]',
    description='Challenge 4 - Multi Puzzlebot Simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_pub = multi_puzzlebot_sim.joint_state_pub:main',
            'puzzlebot_sim = multi_puzzlebot_sim.puzzlebot_sim:main',
            'localisation = multi_puzzlebot_sim.localisation:main',
            'point_stabilisation_control = multi_puzzlebot_sim.point_stabilisation_control:main',
        ],
    },
)