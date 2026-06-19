#pragma once
#include <Eigen/Dense>
#include <vector>

#include "two_view.h"  // Intrinsics

namespace webslam {

// A camera pose (world -> camera).
struct BACamera {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  bool fixed = false;  // fixed cameras anchor the gauge (e.g. the first keyframe)
};

// A reprojection observation: camera `cam` sees point `pt` at pixel `uv`.
struct BAObservation {
  int cam;
  int pt;
  Eigen::Vector2d uv;
};

// A bundle-adjustment problem: cameras, 3D points, and the observations linking
// them. Optimized in place by bundleAdjust().
struct BAProblem {
  std::vector<BACamera> cameras;
  std::vector<Eigen::Vector3d> points;
  std::vector<BAObservation> obs;
  Intrinsics K;
};

struct BAStats {
  double initialCost = 0;  // mean reprojection error (px) before
  double finalCost = 0;    // mean reprojection error (px) after
  int iterations = 0;
};

// Joint nonlinear refinement of all (non-fixed) camera poses and 3D points,
// minimizing robustified reprojection error. Levenberg-Marquardt with a
// Schur-complement reduction over the camera block (the structure Ceres
// exploits for BA). At least one camera should be fixed to anchor the gauge.
BAStats bundleAdjust(BAProblem& problem, int maxIterations = 10, double huberPx = 2.0);

}  // namespace webslam
