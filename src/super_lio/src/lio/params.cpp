

#include "lio/params.h"

using namespace std;
using namespace BASIC;

namespace LI2Sup{

  const std::string g_root_dir = std::string(ROOT);
  std::atomic<bool> g_flag_run = true; 
  bool g_flg_map_init = true;

  /// evaluation
  bool g_time_eva = false;

  bool   g_save_map;
  bool   g_if_filter; 
  string g_save_map_dir;
  string g_map_name;
  float  g_map_ds_size;
  int    g_pcd_save_interval;
  
  string g_imu_topic;
  string g_lidar_topic;

  int    g_lidar_type;
  float  g_blind2;
  float  g_maxrange2;
  int    g_filter_rate;
  bool   g_enable_downsample;
  float  g_voxel_fliter_size;

  int    g_imu_type;
  double g_gravity_norm = 9.7946;
  double g_imu_na;
  double g_imu_ng;
  double g_imu_nba;
  double g_imu_nbg;

  SE3 g_lidar_imu;
  SE3 g_odom_robo;
  M3  g_lidar_robo_yaw;

  /// hash_map
  std::size_t g_ivox_capacity = 100000;
  float       g_ivox_resolution = 0.5;

  /// kf
  int g_kf_type = 1;                // 1: ESKF, 2: InESKF
  int g_kf_max_iterations = 4;
  bool g_kf_align_gravity = true;
  double g_kf_quit_eps;

  /// submap 
  double g_submap_resolution;
  int    g_submap_capacity;

  /// output
  bool g_2_robot    = false;
  bool g_2_plan_env_world = false; 
  bool g_2_plan_env_body  = false;
  bool g_2_ml_map = false;
  bool g_visual_map = true;
  bool g_visual_dense = false;
  int  g_pub_step;

  /// for planner
  bool g_planner_enable;

  ResidualType g_residual_type = PROB;

  /// for relocation
  bool g_update_map = false;
  double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;

  int g_health_min_effective_points = 20;
  double g_health_min_overlap_ratio = 0.02;
  double g_health_max_mean_residual = 0.30;
  int g_health_degraded_after_bad_frames = 2;
  int g_health_lost_after_bad_frames = 15;
  int g_health_recover_after_good_frames = 3;

  double g_reloc_local_search_radius = 15.0;
  double g_reloc_local_search_z = 3.0;
  double g_reloc_source_voxel_size = 0.20;
  double g_reloc_target_voxel_size = 0.20;
  double g_reloc_max_correspondence_distance = 1.5;
  double g_reloc_hypothesis_xy_offset = 0.5;
  double g_reloc_hypothesis_yaw_deg = 10.0;
  double g_reloc_primary_accept_fitness = 0.20;
  double g_reloc_max_fitness = 0.60;
  double g_reloc_min_overlap_ratio = 0.15;
  double g_reloc_max_seed_translation = 2.0;
  double g_reloc_max_seed_rotation_deg = 30.0;

}
