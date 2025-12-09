from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Configuration VESC
    vesc_config = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config.yaml'
    )

    return LaunchDescription([
        # 1. Joy Node (Lecture de la manette)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0'}] # Vérifier si c'est js0 ou js1
        ),

        # 2. Teleop Node (Conversion Joy -> AckermannDriveStamped sur topic 'drive')
        Node(
            package='mushr_joy_teleop',
            executable='joy_teleop',
            name='joy_teleop_node',
            parameters=[
                {'max_speed': 2.0},
                {'max_steer': 0.34}
            ]
        ),

        # 3. Ackermann to VESC (Conversion Ackermann -> VESC commands)
        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            parameters=[
                {'speed_to_erpm_gain': 4614.0}, # Exemple, à ajuster selon votre moteur
                {'speed_to_erpm_offset': 0.0},
                {'steering_angle_to_servo_gain': -1.2135}, # Exemple
                {'steering_angle_to_servo_offset': 0.5304} # Exemple
            ],
            remappings=[
                ('ackermann_cmd', 'drive') # On connecte la sortie du teleop à l'entrée de ce noeud
            ]
        ),

        # 4. VESC Driver (Communication avec le hardware VESC)
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[vesc_config]
        )
    ])
