from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # GStreamer pipelines for left & right cameras
    gscam_left_config = (
        'nvarguscamerasrc sensor-id=0 ! '
        'video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! '
        'nvvidconv flip-method=0 ! '
        'video/x-raw, width=640, height=480, format=RGBA ! '
        'videoconvert ! '
        'video/x-raw, format=RGB'
    )

    gscam_right_config = (
        'v4l2src device=/dev/video1 ! '
        'video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! '
        'videoconvert ! appsink'
    )

    # Calibration files (update these paths to match your package)
    left_info_url = 'package://imx219_83_stereo_camera/calibration/left.yaml'
    right_info_url = 'package://imx219_83_stereo_camera/calibration/right.yaml'

    container = ComposableNodeContainer(
        name='stereo_gscam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # multithreaded container
        composable_node_descriptions=[
            # --- LEFT CAMERA ---
            ComposableNode(
                package='gscam',
                plugin='gscam::GSCam',
                name='gscam_left',
                namespace='stereo/left',
                parameters=[{
                    'gscam_config': gscam_left_config,
                    'camera_info_url': left_info_url,
                    'frame_id': 'stereo_left_frame',
                    'use_gst_timestamps': True,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            # --- RIGHT CAMERA ---
            ComposableNode(
                package='gscam',
                plugin='gscam::GSCam',
                name='gscam_right',
                namespace='stereo/right',
                parameters=[{
                    'gscam_config': gscam_right_config,
                    'camera_info_url': right_info_url,
                    'frame_id': 'stereo_right_frame',
                    'use_gst_timestamps': True,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            # --- RECTIFICATION ---
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_left',
                namespace='stereo/left',
                remappings=[
                    ('image', 'image_raw'),
                    ('image_rect', 'image_rect'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_right',
                namespace='stereo/right',
                remappings=[
                    ('image', 'image_raw'),
                    ('image_rect', 'image_rect'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            # --- STEREO IMAGE PROCESSING ---
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::DisparityNode',
                name='disparity_node',
                namespace='stereo',
                remappings=[
                    ('left/image_rect', 'left/image_rect'),
                    ('right/image_rect', 'right/image_rect'),
                    ('left/camera_info', 'left/camera_info'),
                    ('right/camera_info', 'right/camera_info'),
                    ('disparity', 'disparity'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
