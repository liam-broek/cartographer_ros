-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,

  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  use_pose_extrapolator = true,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.1,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-4,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1,
}


-- inputs

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1 
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 0.1 
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 250. 
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 1.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 400.

TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 1.

TRAJECTORY_BUILDER_3D.submaps.high_resolution = .1 
TRAJECTORY_BUILDER_3D.submaps.low_resolution = .20 
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 20  

TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.6
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.4
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.num_free_space_voxels = 10

-- local slam

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5   
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 2e2 
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 100 
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 30.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 1.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = true

TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false

-- global slam

POSE_GRAPH.constraint_builder.sampling_ratio = 0.03  --change 0.03
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66
POSE_GRAPH.constraint_builder.min_score = 0.5
POSE_GRAPH.global_sampling_ratio = 0.03

-- pose graph

POSE_GRAPH.optimize_every_n_nodes = 5 
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 200
POSE_GRAPH.optimization_problem.huber_scale = 1e2 
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = false

return options
