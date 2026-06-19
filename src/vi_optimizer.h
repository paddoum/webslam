#pragma once
#include <Eigen/Dense>
#include <vector>

#include "imu_preint.h"

namespace webslam {

// Full visual-inertial state of one keyframe.
struct ViState {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // body→world
  Eigen::Vector3d p = Eigen::Vector3d::Zero();      // position (world, metric)
  Eigen::Vector3d v = Eigen::Vector3d::Zero();      // velocity (world)
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();     // gyro bias
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();     // accel bias
};

// A visual pose measurement for a keyframe (e.g. from PnP against the map).
struct VisualPose {
  bool valid = false;            // false during a tracking dropout
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
};

struct ViOptStats { double initCost = 0, finalCost = 0; int iterations = 0; };

// Sliding-window VI optimization. Jointly refines all keyframe states by
// minimizing: visual pose residuals (where valid) + IMU preintegration
// residuals between consecutive keyframes + a bias random-walk prior. Gravity
// is fixed (from VI init). Levenberg–Marquardt with numerical Jacobians on the
// manifold. `pre[k]` is the preintegration between keyframe k and k+1.
ViOptStats viOptimize(std::vector<ViState>& states,
                      const std::vector<Preintegrated>& pre,
                      const std::vector<VisualPose>& vis,
                      const Eigen::Vector3d& gravity, int iterations = 20);

}  // namespace webslam
