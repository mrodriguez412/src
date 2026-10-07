from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    detector_node = launch_ros.actions.Node(
        package='cylinder_detector',
        executable='cylinder_detector',
        name='cylinder_detector',
        parameters=[
            {'target_frame': 'world'},
            # Point selection
            {'min_range': 0.4},             # m, rejects the robot body and near noise
            {'max_range': 3.5},             # m, rejects noisy long-range points
            {'floor_z': 0.0},               # m, floor height in target_frame
            {'floor_clearance': 0.05},      # m, points below floor_z + floor_clearance are floor
            {'max_z': 2.0},                 # m
            {'voxel_size': 0.02},           # m, one point kept per voxel
            {'cluster_resolution': 0.05},   # m, grid cell size used to cluster the obstacles
            # Circle RANSAC
            {'n_samples': 200},
            {'tolerance': 0.015},           # m, max distance to the circle for an inlier
            {'min_radius': 0.05},           # m
            {'max_radius': 0.5},            # m
            {'min_inliers': 50},
            {'max_fits_per_cluster': 3},
            # Cylinder validation
            {'min_inlier_ratio': 0.7},      # inliers / points in the ring around the circle
            {'support_margin': 0.1},        # m, outer width of that ring
            {'min_arc_angle': 1.5708},      # rad (90 deg), arc covered by the inliers
            {'min_height_coverage': 0.8},   # inlier height / height of the points over the disc
            {'min_height': 0.1},            # m
            # Map of cylinders
            {'assoc_distance': 0.3},        # m, max distance between centres to merge
            {'min_observations': 3},        # detections needed before publishing a cylinder
        ],
        output='screen',
    )

    return LaunchDescription([
        detector_node,
    ])
