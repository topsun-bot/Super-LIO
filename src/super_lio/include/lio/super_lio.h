

#ifndef SUPER_LIO_H_
#define SUPER_LIO_H_

#include <cstdint>
#include <queue>
#include <vector>
#include <iostream>
#include <cassert>
#include <filesystem>
#include <limits>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "basic/alias.h"
#include "common/ds.h"
#include "params.h"
#include "ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"
#include "OctVoxMap/VoxelGridFilter.h"
#include "ros/ROSWrapper.h"

namespace LI2Sup{

enum class LocalizationState : std::uint8_t {
  INITIALIZING = 0,
  RELOCALIZING = 1,
  TRACKING = 2,
  DEGRADED = 3,
  LOST = 4,
};

struct RegistrationQuality {
  bool update_accepted = false;
  std::size_t input_points = 0;
  std::size_t effective_points = 0;
  double overlap_ratio = 0.0;
  double mean_abs_residual = std::numeric_limits<double>::infinity();
};

class SuperLIO{
public:
  SuperLIO(){};
  ~SuperLIO(){};

  void setROSWrapper(const ROSWrapper::Ptr& wrapper){
    data_wrapper_ = wrapper;
  }
  virtual void init();
  void process();
  void saveMap();

protected:
  void stateWaitKFInit();
  void stateWaitMapInit();
  void stateProcess();
  virtual bool kf_init();
  virtual bool map_init();
  void Propagation_Undistort();
  void DownSample();
  void Observe();
  virtual void UpdateMap();
  virtual void Output();
  void caceData();
  void ProcessCaceMap();
  void updateLocalizationHealth();
  void publishLocalizationStatus();
  void setLocalizationState(LocalizationState state);
  static const char* localizationStateName(LocalizationState state);

  using StateFn = void (SuperLIO::*)();
  using OctVoxMapType = OctVoxMap<BASIC::V3, BASIC::scalar>;
  using KNNHeapType = KNNHeap<5, BASIC::V3>;
  StateFn state_fn_;
  ESKF::Ptr kf_;
  OctVoxMapType::Ptr ivox_;
  VoxelGridClosest<BASIC::PointType> voxel_grid_fliter_;
  ROSWrapper::Ptr data_wrapper_;
  MeasureGroup measures_;
  
  bool flg_init_ = false;
  bool flg_first_scan_ = true;
  std::vector<DynamicState> propagate_states_;
  BASIC::CloudPtr scan_undistort_full_;
  BASIC::CloudPtr ds_undistort_;
  BASIC::CloudPtr point_map_, world_pc_, ds_world_;
  int frame_num_ = 0;
  BASIC::SE3 sys_init_pose_;
  BASIC::SE3 last_pose_;

  std::size_t effect_knn_num_ = 0;
  BASIC::VV3 points_world_v3_, points_body_v3_;
  std::vector<std::uint8_t> effect_mask_;
  std::vector<std::uint8_t> effect_knn_mask_;
  std::vector<int> effect_knn_idxs_;
  std::vector<std::pair<BASIC::M6, BASIC::V6>> H_R_;
  std::vector<std::array<double, 4>> abcd_vec_;
  int pcd_index_ = -1;

  LocalizationState localization_state_ = LocalizationState::INITIALIZING;
  RegistrationQuality registration_quality_;
  int consecutive_good_frames_ = 0;
  int consecutive_bad_frames_ = 0;
  double initial_alignment_fitness_ = std::numeric_limits<double>::infinity();
};

} // namespace END.

#endif
