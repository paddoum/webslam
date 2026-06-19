#pragma once
#include <Eigen/Dense>
#include <vector>

#include "two_view.h"  // Intrinsics

namespace webslam {

struct PnPResult {
  bool ok = false;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // world -> camera
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  int inliers = 0;
  std::vector<char> inlierMask;
};

// Solve the Perspective-n-Point problem: given 3D world points and their 2D
// pixel observations, recover the camera pose (world -> camera). Uses a
// RANSAC + DLT hypothesis, then Gauss-Newton reprojection refinement on the
// inliers. This is the open analogue of the engine's PosePnP.
//
// If `hasGuess` is true, (Rinit, tinit) seed the refinement (the tracking loop
// always has the previous pose as a prior, which makes this fast and robust).
PnPResult solvePnP(const std::vector<Eigen::Vector3d>& worldPts,
                   const std::vector<Eigen::Vector2d>& imgPts, const Intrinsics& K,
                   double pixelThreshold = 3.0, int iterations = 200,
                   unsigned seed = 12345, bool hasGuess = false,
                   const Eigen::Matrix3d& Rinit = Eigen::Matrix3d::Identity(),
                   const Eigen::Vector3d& tinit = Eigen::Vector3d::Zero());

}  // namespace webslam
