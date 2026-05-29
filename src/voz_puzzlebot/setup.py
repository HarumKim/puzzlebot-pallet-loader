from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'voz_puzzlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- NUEVO: Incluir todos los archivos de tu carpeta Voz_puzzlebot al compilar ---
        (os.path.join('share', package_name, 'voz_puzzlebot'), glob('voz_puzzlebot/*.pkl')),
        (os.path.join('lib', 'python3.10', 'site-packages', 'voz_puzzlebot'),
        ['voz_puzzlebot/codebook_256.pkl','voz_puzzlebot/modelos_hmm_entrenados.pkl',]
    ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alondra',
    maintainer_email='tu_correo@todo.com',
    description='Nodo de reconocimiento de voz HMM para el Puzzlebot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Registramos tu archivo comando_node.py con el comando ejecutable comando_node
            'comando_node = voz_puzzlebot.comando_node:main',
            'fsm_control_node = voz_puzzlebot.fsm_control_node:main',
        ],
    },
)
