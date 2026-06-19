#pragma once
#include <Eigen/Dense>
#include <vector>

#include "imu_preint.h"

namespace webslam {

struct ViInit {
  bool ok = false;
  double scale = 0;                            // metric scale: metres = scale · visual units
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();  // gravity in the first-frame (c0) frame
  double gravityMag = 0;
  std::vector<Eigen::Vector3d> velocities;     // per-keyframe velocity (c0 frame, metric)
};

// Visual-inertial alignment (VINS-Mono style). Given keyframe body→world (=c0)
// rotations and up-to-scale visual positions, plus the IMU preintegration
// between consecutive keyframes, solve a linear system for per-frame velocity,
// gravity, and the metric scale. Assumes camera≈IMU (identity extrinsic) and
// negligible gyro bias (justified by the M12.0 on-device measurement).
//
// Rwb[i]      : body→world rotation of keyframe i (n+1 of them)
// pVisual[i]  : up-to-scale position of keyframe i in the c0 frame
// pre[k]      : preintegration between keyframe k and k+1 (n of them)
ViInit viInitialize(const std::vector<Eigen::Matrix3d>& Rwb,
                    const std::vector<Eigen::Vector3d>& pVisual,
                    const std::vector<Preintegrated>& pre);

}  // namespace webslam
