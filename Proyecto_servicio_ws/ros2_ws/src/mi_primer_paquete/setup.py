from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mi_primer_paquete'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf', 'Modelos_en_stl'), glob('urdf/Modelos_en_stl/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aldo',
    maintainer_email='aldo.solorio.unam@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = mi_primer_paquete.publisher:main',
            'subscriber = mi_primer_paquete.subscriber:main',
            'subscriber_inter = mi_primer_paquete.subscriber_inter:main',
            'subscriber_2 = mi_primer_paquete.subscriber_2:main',
            'punto_objetivo = mi_primer_paquete.punto_objetivo:main',
            'publicador_juntas_terminal = mi_primer_paquete.publicador_juntas_terminal:main',
            'clicked_point_a_juntas = mi_primer_paquete.clicked_point_a_juntas:main',
            'joint_states_a_esp32 = mi_primer_paquete.joint_states_a_esp32:main',
        ],
    },
)
