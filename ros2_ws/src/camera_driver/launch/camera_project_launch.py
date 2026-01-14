from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Uruchomienie węzła kamery
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv'
            }]
        ),
        
      
        Node(
            package='camera_driver',
            executable='camera_node',
            name='controller',
            parameters=[{
                'rect_size': 50  # Tutaj możesz zmienić domyślny rozmiar
            }],
            output='screen' # Dzięki temu zobaczysz logi w terminalu
        )
    ])
