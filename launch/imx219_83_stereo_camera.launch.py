from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = []

    stereo_camera_left = Node(
        package='gscam',
        executable='gscam_node',
        name='stereo_camera_rawimage_publisher_left',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                # Set the name to correspond the camera_info topic name
                'camera_name': 'stereo',

                # Camera calibration file
                'camera_info_url': 'package://imx219_83_stereo_camera/calibration/left.yaml',

                # GStreamer config string
                'gscam_config': 'v4l2src device=/dev/video0 ! video/x-raw,framerate=30 ! videoconvert',
                
                # TF2 frame id
                'frame_id': 'stereo_camera_left',

                # Re-open the stream if it ends (EOF)
                'reopen_on_eof': True,

                # Synchronize the app sink (sometimes setting this to false can
                # resolve problems with sub-par framerates)
                'sync_sink': True,

                # Use the GStreamer buffer timestamps for the image message
                # header timestamps (setting this to false results in header
                # timestamps being the time that the image buffer transfer is
                # completed)
                'use_gst_timestamps': True,

                # “rgb8”, “mono8”, “yuv422”, “jpeg”
                'image_encoding': 'rgb8',
                
                # The flag to use sensor data qos for camera topic(image, camera_info)
                'use_sensor_data_qos': True
            }
        ]

    )
    ld.append(stereo_camera_left)




    return LaunchDescription(ld)
