import os

from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    return LaunchDescription([
        Node(
            name='realsense_depth_camera',
            package='realsense_ros2_camera',
            executable='realsense_ros2_camera',
            output='both',
        ),
        Node(
            name='composable_node_container',
            package='rclcpp_components',
            executable='component_container',
            output='both',
        ),
        LoadComposableNodes(
            target_container="composable_node_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="pcl_ros",
                    plugin='pcl_ros::VoxelGrid', 
                    name='filter_voxel_grid_node', 
                    parameters=[{ 
                        'leaf_size': 0.01, 
                        'min_points_per_voxel': 2, 
                        'filter_limit_max': 2.0, 
                    }], 
                    remappings=[ 
                        ('input', '/camera/aligned_depth_to_color/color/points'), 
                        ('output', '/depth/voxel_grid/output'),
                   ]
                ),
                ComposableNode(
                    package="pcl_ros",
                    plugin='pcl_ros::EuclideanClusterExtraction', 
                    name='EuclideanClusterExtractionNode', 
                    parameters=[{ 
                        'cluster_tolerance': 0.03,
                        'publish_indices': False,
                        'spatial_locator': 1,  # FLANN
                    }], 
                    remappings=[ 
                        ('input', '/depth/voxel_grid/output'), 
                        # ('output', 'TODO'),
                   ]
                )
            ]),
    ])
