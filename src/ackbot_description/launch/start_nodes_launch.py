from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    urdf_file = os.path.join(
        os.getenv('HOME'), 'ackbot_ws/src/ackbot_description/urdf/yahboomcar_R2.urdf')

    return LaunchDescription([
        # Velocity publisher
        Node(
            package='ackbot_velocity',
            executable='vel_raw_pub',
            name='vel_raw_pub',
            output='screen'
        ),
        # Odometry publisher
        Node(
            package='ackbot_velocity',
            executable='odom_pub',
            name='odom_pub',
            output='screen'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_width': 640},
                {'image_height': 480},
                {'pixel_format': 'yuyv'},  
                {'camera_frame_id': 'camera'},
                {'framerate': 15.0},
                {'brightness': 128},                    # Final brightness
                {'contrast': 75},                       # Final contrast
                {'sharpness': 255},                     # Final sharpness
                {'auto_exposure': 3},                   # Final auto exposure (Aperture Priority Mode)
                {'exposure_time_absolute': 500},        # Final exposure time
                {'white_balance_automatic': 1},         # Final white balance automatic
            ]
        ),
        # LIDAR node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('ldlidar_stl_ros2'), '/launch/ld19.launch.py'
            ])
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open('/home/ubuntu/ackbot_ws/src/ackbot_description/urdf/yahboomcar_R2.urdf').read()}]
        ),
        # OLED display node
        Node(
            package='ackbot_display',
            executable='display',
            name='battery_display',
            output='screen'
        ),
    ])

