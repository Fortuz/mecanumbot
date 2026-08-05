-- Copyright 2016 The Cartographer Authors
-- Licensed under the Apache License, Version 2.0

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  
  -- 1. FRAME FIX: Track base_link instead of an unused IMU link
  tracking_frame = "mecanumbot/base_link",
  published_frame = "mecanumbot/odom",
  odom_frame = "mecanumbot/odom",
  
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.5,
  
  -- 2. CPU FIX: Reduced pose publishing from 200 Hz down to 20 Hz
  pose_publish_period_sec = 5e-2,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  -- Full rate: odom is cheap, and with use_imu_data=false it is the *only* input
  -- to the pose extrapolator. Halving it degrades the scan-match prior for
  -- nothing, and mecanum slip already makes that prior the weak link.
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 3. THREAD FIX: Explicitly cap background threads to stop CPU thrashing
MAP_BUILDER.num_background_threads = 4

-- LD08 / LDS-02, not the LDS-01 this was copied from: 0.16 m .. 12 m nominal.
-- Capped at 8 m because long returns off low-reflectivity surfaces get noisy,
-- and 8 m already spans the ~10 x 9 m arena from most of it.
TRAJECTORY_BUILDER_2D.min_range = 0.16
TRAJECTORY_BUILDER_2D.max_range = 8.0

-- 4. RAY LENGTH: must be <= max_range, NOT greater.
-- A beam longer than max_range is not dropped - AddRangeData turns it into a
-- *miss* re-cast at missing_data_ray_length, and misses raytrace free space.
-- With max_range 3.5 / ray length 5.0 every scan repainted the 3.5-5.0 m
-- annulus as free, erasing walls already mapped from closer up. Keeping this
-- shorter than max_range means no-return beams only clear the near field.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 5. MOTION FILTER: ignore redundant scans when the robot is stationary.
-- 0.1 deg is below the pose noise floor, so it filtered almost nothing and the
-- graph grew a node per scan. Back to the upstream 0.2 m / 1 deg, which is what
-- pays for the larger submaps below.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0

-- 6. SUBMAP DENSITY: occupancy_grid_node composites submaps in index order, so a
-- newer submap overwrites an older one where they overlap. At 35 a submap closes
-- after 70 scans, under-converged and pale, and paints over a well-observed
-- neighbour - walls go grey. This was labelled a RAM fix, but it cuts memory the
-- wrong way: fewer range data per submap means MORE submaps, and Cartographer's
-- footprint scales with submap count. Drop to 60 if the Orin still runs tight.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.optimize_every_n_nodes = 60

return options