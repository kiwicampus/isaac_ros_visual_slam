# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import launch
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """Launch file to bring up visual slam node standalone."""
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        # remappings=[('/stereo_camera/left/camera_info', '/camera_info_left'),
        #             ('/stereo_camera/right/camera_info', '/camera_info_right')],
        parameters=[{
                    'use_sim_time': True,
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_slam_visualization': True,
                    'enable_observations_view': True,
                    'enable_landmarks_view': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'map_frame': 'visual_map',
                    'odom_frame': 'visual_odom',
                    'base_frame': 'base_link',
                    'input_base_frame': 'base_link',

                    'input_imu_frame': 'camera_gyro_optical_frame',
                    'enable_imu_fusion': False,
                    'gyro_noise_density': 0.000244,
                    'gyro_random_walk': 0.000019393,
                    'accel_noise_density': 0.001862,
                    'accel_random_walk': 0.003,
                    'calibration_frequency': 200.0,
                    'img_jitter_threshold_ms': 22.00,

                    'input_left_camera_frame': 'camera_infra1_frame',
                    # 'input_right_camera_frame': 'camera_infra2_frame',
                    'path_max_size': 4096*2,
                    'img_jitter_threshold_ms': 100.0,
                    'force_planar_mode': False,
                    'enable_localization_n_mapping': False,
                    'publish_odom_to_base_tf': False
                    }],
        remappings=[('stereo_camera/left/image', 'camera/infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info', 'camera/infra1/camera_info'),
                    ('stereo_camera/right/image', 'camera/infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', 'camera/infra2/camera_info'),
                    ('visual_slam/imu', 'camera/imu')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )

    map_to_visual_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_visual_map",
        # output="screen",
        arguments=[
            "60.07978058",  # x
            "133.1325531",  # y
            "-0.55089456",  # z
            "0.0",   # Quaternion x
            "0.0",          # Quaternion y
            "0.5318392",          # Quaternion z
            "0.846744",   # Quaternion w
            "map",          # Parent frame
            "visual_map"    # Child frame
        ]
    )

    # map_to_visual_map = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="map_to_visual_map",
    #     # output="screen",
    #     arguments=[
    #         "60.07978058",  # x
    #         "133.1325531",  # y
    #         "-0.55089456",  # z
    #         "0.0",   # Quaternion x
    #         "0.0",          # Quaternion y
    #         "0.532"          # Quaternion z
    #         "0.847",   # Quaternion w
    #         "map",          # Parent frame
    #         "visual_map"    # Child frame
    #     ]
    # )

    navsat_transform = Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[os.path.join('/workspaces/isaac_ros-dev/config', 'navsat_transform.yaml')],
            remappings=[
                    ("imu", "/imu/data"),
                    ("gps/fix", "/fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
    )

    return launch.LaunchDescription([
        visual_slam_launch_container, 
        # map_to_visual_map, 
        navsat_transform,
        ])
