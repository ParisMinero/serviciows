from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1) Buscar la carpeta instalada del paquete
    pkg_share = get_package_share_directory('mi_primer_paquete')

    # 2) Construir la ruta al archivo URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'prueba_2.urdf')

    # 3) Leer el contenido completo del URDF como texto
    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()
    

    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mi_robot.rviz')

    # 4) Crear el nodo robot_state_publisher y pasarle el URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content}
        ]
    )

    # 5) Abrir RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # 6) Devolver todos los nodos que lanzará este archivo
    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])