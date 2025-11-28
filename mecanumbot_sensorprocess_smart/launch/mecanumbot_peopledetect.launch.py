leg_detector_path = get_package_share_directory('leg_detector')
description_path = get_package_share_directory('mecanumbot_description')

rviz2_config_path = os.path.join(description_path,'rviz','model.rviz')
leg_detector_config_path = os.path.join(
    leg_detector_path, "config", "trained_leg_detector_res=0.33.yaml"
)

def generate_launch_description():

    ld = LaunchDescription([

        # Launching RVIZ2 (optional namespace)
        launch.actions.ExecuteProcess(
            cmd=[
                'ros2', 'run', 'rviz2', 'rviz2',
                '-d', rviz2_config_path
            ],
            output='screen'
        )
    ])

    # Detect leg clusters
    detect_leg_clusters_node = Node(
        package="leg_detector",
        executable="detect_leg_clusters",
        name="detect_leg_clusters",
        namespace="mecanumbot",
        parameters=[
            {"forest_file": leg_detector_config_path},
            {"scan_topic": "scan"},     # becomes /mecanumbot/scan internally
            {"fixed_frame": "laser"},
        ]
    )

    # Joint leg tracker
    joint_leg_tracker_node = Node(
        package="leg_detector",
        executable="joint_leg_tracker.py",
        name="joint_leg_tracker",
        namespace="mecanumbot",
        parameters=[
            {"scan_topic": "scan"},     # → /mecanumbot/scan
            {"fixed_frame": "laser"},
            {"scan_frequency": 10},
        ]
    )

    # Inflated human scan
    inflated_human_scan_node = Node(
        package="leg_detector",
        executable="inflated_human_scan",
        name="inflated_human_scan",
        namespace="mecanumbot",
        parameters=[
            {"inflation_radius": 1.0}
        ]
    )

    # Local occupancy grid mapping
    local_occupancy_grid_mapping_node = Node(
        package="leg_detector",
        executable="local_occupancy_grid_mapping",
        name="local_occupancy_grid_mapping",
        namespace="mecanumbot",
        parameters=[
            {"scan_topic": "scan"},   # → /mecanumbot/scan
            {"fixed_frame": "laser"},
        ]
    )

    ld.add_action(detect_leg_clusters_node)
    ld.add_action(joint_leg_tracker_node)
    ld.add_action(inflated_human_scan_node)
    ld.add_action(local_occupancy_grid_mapping_node)

    return ld
