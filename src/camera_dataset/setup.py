from setuptools import find_packages, setup

package_name = 'camera_dataset'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='harumkim09@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dataset_creator = camera_dataset.dataset_node:main',
            'camera_node= camera_dataset.camera_node:main',
            'qr_distance = camera_dataset.qr_distance:main'
        ],
    },
)
