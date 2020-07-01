include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "create" ..                    -- Frame id to track from map
                   os.getenv("ID") ..
                   "_tf/base_footprint",
  published_frame = "create" ..                   -- Odom frame to publish if `provide_odom_frame = true`. Even though
                    os.getenv("ID") ..            -- are not used, all the options' parameter keys are mandatory
                    "_tf/odom",
  odom_frame =      "create" ..                   -- Frame where publish the odometry to if `provide_odom_frame=true`
                    os.getenv("ID") ..
                    "_tf/odom",
  provide_odom_frame = false,                     -- Request cartographer to provide `odom_frame`
  publish_frame_projected_to_2d = true,           -- Projects any data to the 2D plane
  use_odometry = true,                            -- Uses the odometry published on /odom. Particularly in this case
                                                  -- it's being already provided by DiffDrive
  use_nav_sat = false,                            -- Uses GPS data in form of sensor_msgs/NavSatFix
                                                  -- published on topic /fix
  use_landmarks = false,                          -- Reads `cartogreapher_ros_msgs/LandmarkList` messages with landmarks
                                                  -- published in the topic /landmarks with respect to the tracking frame
  num_laser_scans = 1,                            -- Number of laser scan topics provided through /scan. If the amount
                                                  -- is more than 1, the topics will be /scan_1 /scan_2 /scan_n
  num_multi_echo_laser_scans = 0,                 -- Number of topics publishing multi echo laser scans. They have to
                                                  -- be published on /echoes, and if more than one it follows the same
                                                  -- than num_laser_scans
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  use_pose_extrapolator = true,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = false        -- Avoid reading imu data when doing SLAM 2D. Scan matching is prefered

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true     -- Use Ceres scan matching
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)

TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2.0

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 70
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 300

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.035    -- Resolution in meters per grid cell
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120
POSE_GRAPH.optimize_every_n_nodes = 120
POSE_GRAPH.constraint_builder.min_score = 0.82
POSE_GRAPH.constraint_builder.sampling_ratio = 1.

POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options
