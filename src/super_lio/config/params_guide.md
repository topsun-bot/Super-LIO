# Super-LIO 配置文件参数说明

> 配置文件：`rsairy.yaml` | 代码版本基于 `ROSWrapper.cpp` 参数加载逻辑

---

## 1. lio.map — 地图保存

| 参数 | 类型 | 默认值 | 当前值 | 说明 |
|------|------|--------|--------|------|
| `lio.map.save_map` | bool | false | true | 是否在退出时保存地图。关闭后不会触发 `saveMap()` |
| `lio.map.if_filter` | bool | false | true | 保存地图时是否使用体素滤波降采样。`true` 用 `ds_undistort_`（降采样后的点云），`false` 用原始全量点云 |
| `lio.map.save_map_dir` | string | "" | "map" | 地图保存目录（相对路径）。实际路径 = `g_root_dir` + 该值 = `src/super_lio/map/` |
| `lio.map.map_name` | string | "default" | "map.pcd" | 最终合并输出的地图文件名 |
| `lio.map.ds_size` | double | 0.5 | 0.2 | 保存地图时体素滤波的栅格尺寸（米）。值越小密度越高、文件越大 |
| `lio.map.save_interval` | int | 1 | 100 | 每隔多少帧激光雷达扫描将缓存的点云保存到 `PCD/scans_N.pcd`。<0 时不保存碎片、退出时直接存整张地图；>0 时边跑边存碎片、退出时合并生成最终文件 |

**保存时机**：`save_interval > 0` 时每 N 帧保存一次碎片到 `PCD/`；程序退出（Ctrl+C）时自动调用 `saveMap()` → `ProcessCaceMap()` 合并碎片并下采样输出最终 `.pcd`。

---

## 2. lio.ros — ROS 话题

| 参数 | 类型 | 默认值 | 当前值 | 说明 |
|------|------|--------|--------|------|
| `lio.ros.lidar_topic` | string | "/lidar" | "/rslidar_points" | 订阅的激光雷达点云话题名 |
| `lio.ros.imu_topic` | string | "/imu" | "/rslidar_imu_data" | 订阅的 IMU 数据话题名 |

---

## 3. lio.sensor — 传感器参数

| 参数 | 类型 | 默认值 | 当前值 | 说明 |
|------|------|--------|--------|------|
| `lio.sensor.lidar_type` | int | 0 | 2 | 激光雷达型号。1=LIVOX(专用CustomMsg), **2=HESAI16(标准PointCloud2,适用于rslidar)**, 3=VELO16, 4=VELO32, 5=VEL_NCLT, 6=LS16(预留), 7=OUSTER |
| `lio.sensor.blind` | double | 0.0 | 0.2 | 盲区距离（米）。距离小于此值的点被丢弃。内部存平方值用于快速比较 |
| `lio.sensor.maxrange` | double | 100.0 | 60.0 | 最大有效距离（米）。超过此值的点被丢弃。内部存平方值 |
| `lio.sensor.filter_rate` | int | 1 | 3 | 点云步长降采样。每隔 N 个点取 1 个（`i += filter_rate`），在进入点云处理前先砍掉点。1=不跳过 |
| `lio.sensor.enable_downsample` | bool | false | true | 是否启用体素滤波降采样（在点云配准前） |
| `lio.sensor.voxel_fliter_size` | double | 0.2 | 0.5 | 体素滤波栅格尺寸（米）。仅在 `enable_downsample=true` 时生效 |
| `lio.sensor.gravity_norm` | double | 9.81 | 9.7946 | 当地重力加速度大小（m/s²）。用于 IMU 尺度自动校准：`imu_scale = gravity_norm / mean_acce.norm()`，归一化IMU数据会被自动恢复到m/s² |
| `lio.sensor.imu_type` | int | 0 | 1 | **（预留，当前未使用）** IMU 型号标识。所有 IMU 统一使用 `sensor_msgs/msg/Imu` 处理 |
| `lio.sensor.imu_na` | double | 0.0 | 0.1 | IMU 加速度计噪声标准差（m/s²）。用于 ESKF 噪声建模 |
| `lio.sensor.imu_ng` | double | 0.0 | 0.1 | IMU 陀螺仪噪声标准差（rad/s）。用于 ESKF 噪声建模 |
| `lio.sensor.imu_nba` | double | 0.0 | 0.0001 | IMU 加速度计 bias 随机游走标准差。用于 ESKF bias 估计 |
| `lio.sensor.imu_nbg` | double | 0.0 | 0.0001 | IMU 陀螺仪 bias 随机游走标准差。用于 ESKF bias 估计 |

---

## 4. lio.extrinsic — 外参

| 参数 | 类型 | 当前值 | 说明 |
|------|------|--------|------|
| `lio.extrinsic.lidar_imu` | float[12] | 12 个值 | LiDAR → IMU 的外参变换矩阵。**前 3 个**是平移 t(x,y,z)（米），**后 9 个**是 3×3 旋转矩阵 R（以行优先展开），表示 **LiDAR 坐标系在 IMU 坐标系下的位姿**：`p_imu = R * p_lidar + t` |
| `lio.extrinsic.odom_robo` | float[6] | 全 0 | 里程计/IMU → 机器人基座的外参。`[x, y, z, roll, pitch, yaw]`，角为度。当 IMU 与机器人中心有偏移时需设置 |

**当前 lidar_imu 外参解读**（第 2 组，非注释组）：
```
平移: [0.00425, 0.00418, -0.00446] m
旋转:  3x3 矩阵（近似单位阵，略有微小偏角）
```
这是一个经过标定的精细外参，平移分量仅在 4mm 量级。

---

## 5. lio.hash_map — 哈希体素地图 (iVox)

| 参数 | 类型 | 默认值 | 当前值 | 说明 |
|------|------|--------|--------|------|
| `lio.hash_map.hash_capacity` | int | 1000000 | 2000000 | 哈希表最大容量（体素数量）。根据场景大小和环境调整，越大占用内存越多 |
| `lio.hash_map.vox_resolution` | double | 0.5 | 0.5 | 体素分辨率（米）。决定局部地图中每个体素的边长。越小精度越高但计算/内存开销越大 |

Super-LIO 使用 **iVox（增量体素）** 数据结构管理局部地图，基于空间哈希实现 O(1) 近邻查询。

---

## 6. lio.kf — 卡尔曼滤波器 (ESKF)

| 参数 | 类型 | 默认值 | 当前值 | 说明 |
|------|------|--------|--------|------|
| `lio.kf.kf_type` | int | 1 | 0 | **（预留，当前未实际使用）** 卡尔曼滤波类型。注释：1=ESKF, 2=InESKF |
| `lio.kf.kf_max_iterations` | int | 0 | 4 | ESKF 观测更新时的最大迭代次数。迭代越多精度越高但耗时越大 |
| `lio.kf.kf_align_gravity` | bool | false | true | 是否在初始化时将重力方向对齐到世界坐标系的 Z 轴 |
| `lio.kf.kf_quit_eps` | double | 0.0 | 0.001 | ESKF 迭代收敛阈值。当状态增量小于此值时提前退出迭代 |

ESKF（Error-State Kalman Filter）融合 IMU 预测与 LiDAR 观测，是系统的核心状态估计器。

---

## 7. lio.output — 输出与可视化

| 参数 | 类型 | 默认值 | 当前值 | 说明 |
|------|------|--------|--------|------|
| `lio.output.robot` | bool | false | true | 是否向 `/mavros/vision_pose/pose` 发布机器人位姿（用于 PX4 等飞控的视觉定位） |
| `lio.output.plan_env_world` | bool | false | false | **（预留）** 是否输出世界坐标系下的规划环境点云 |
| `lio.output.plan_env_body` | bool | false | false | **（预留）** 是否输出机体坐标系下的规划环境点云 |
| `lio.output.ml_map` | bool | false | false | **（预留）** 是否输出用于机器学习的点云地图 |
| `lio.output.planner` | bool | false | false | **（预留）** 是否启用规划器相关输出 |
| `lio.output.map` | bool | false | true | 是否发布全局点云地图话题 `/lio/cloud_world` |
| `lio.output.dense` | bool | false | true | 发布地图时是否使用稠密点云（原始未降采样点云）。`true`=原始点云, `false`=体素降采样后点云 |
| `lio.output.pub_step` | int | 0 | 1 | 每隔多少帧发布一次可视化点云。1=每帧发布，N=每N帧发布一次 |

**系统固定发布的 topic**（不受以上参数控制）：
- `/lio/odom` — 激光里程计（LiDAR 频率）
- `/lio/imu/odom` — IMU 预测里程计（IMU 频率）
- `/lio/robo/odom` — 机器人基座里程计
- `/lio/path` — 轨迹路径（点间隔 > 0.1m 时追加新点）
- `tf: world → imu` — IMU 坐标系变换

---

## 8. lio.submap — 子地图（预留）

| 参数 | 类型 | 当前值 | 说明 |
|------|------|--------|------|
| `lio.submap.submap_resolution` | double | — | **（预留，未使用）** |
| `lio.submap.submap_capacity` | int | — | **（预留，未使用）** |

---

## 9. lio.relocation — 重定位（预留）

| 参数 | 类型 | 说明 |
|------|------|------|
| `lio.relocation.update_map` | bool | 重定位时是否更新地图 |
| `lio.relocation.init_pose` | float[6] | 重定位初始位姿 `[x, y, z, roll, pitch, yaw]` |

这些参数在 `super_lio_node` 中未使用，仅在 `relocation_node` 中使用。

---

## 10. lio.eva — 性能评估

| 参数 | 类型 | 当前值 | 说明 |
|------|------|--------|------|
| `lio.eva.timer` | bool | true | 是否打印各模块耗时统计（退出时输出）。关闭后 `printTimeRecord()` 不输出 |

---

## 参数间关系速查

```
原始点云 (rslidar_points)
  │
  ├─ blind + maxrange ──── 距离过滤
  ├─ filter_rate ─────────── 步长降采样 (每 N 取 1)
  ├─ lidar_type ──────────── 解析点云格式 (HESAI16)
  ├─ enable_downsample ──── 体素滤波 (voxel_fliter_size)
  │
  └─ 进入 LiDAR-IMU 里程计
       │
       ├─ ESKF Predict: IMU (imu_na/ng/nba/nbg) → 状态预测
       │    └─ imu_scale = gravity_norm / mean_acce.norm()
       ├─ iVox (vox_resolution, hash_capacity) → 近邻搜索
       ├─ ESKF Update (kf_max_iterations, kf_quit_eps) → 观测更新
       │
       └─ 输出: odom, path, tf, cloud_world (output.map/dense/pub_step)
```
