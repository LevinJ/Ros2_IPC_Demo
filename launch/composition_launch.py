"""
Dynamically compose GSCamNode and ImageSubscriberNode in a component_container.

Limitations:
 -- stdout is not set to flush after each line, so the most recent log messages may be delayed
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    gscam_config = 'videotestsrc pattern=snow ! video/x-raw,width=1280,height=720 ! videoconvert'

    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='cpp_pubsub',
                plugin='MinimalPublisher',
                name='image_publisher',
                # parameters=[{
                #     'gscam_config': gscam_config,
                # }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='cpp_pubsub',
                plugin='MinimalSubscriber',
                name='image_subscriber',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='cpp_pubsub',
                plugin='MinimalSubscriber',
                name='image_subscriber2',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
