#pragma once
#include <Eigen/Dense>
#include <vector>

namespace webslam {

// Pinhole camera intrinsics.
struct Intrinsics {
  double fx, fy, cx, cy;
};

// Result of two-view relative pose estimation between frame A and frame B.
struct RelativePose {
  bool ok = false;            // did we recover a valid pose?
  Eigen::Matrix3d R;          // rotation of B relative to A
  Eigen::Vector3d t;          // unit translation direction (scale unknown)
  int inliers = 0;            // RANSAC inliers supporting this pose
  std::vector<char> inlierMask;  // per-correspondence inlier flag
};

// A 2D-2D point correspondence in pixel coordinates.
struct Match2D {
  Eigen::Vector2d a;  // pixel in frame A
  Eigen::Vector2d b;  // pixel in frame B
};

// Estimate the relative camera pose from 2D-2D correspondences using a
// RANSAC + normalized 8-point essential matrix, then recover (R, t) by SVD
// decomposition with a cheirality (points-in-front) check.
//
// `seed` makes RANSAC deterministic (no global RNG — important for WASM/resume).
RelativePose estimateRelativePose(const std::vector<Match2D>& matches,
                                  const Intrinsics& K,
                                  double pixelThreshold = 1.5,
                                  int iterations = 200,
                                  unsigned seed = 12345);

// Linear (DLT) triangulation of one point given normalized camera coords and
// the two projection matrices P1=[I|0], P2=[R|t]. Returns the 3D point.
Eigen::Vector3d triangulate(const Eigen::Vector2d& xn1, const Eigen::Vector2d& xn2,
                            const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

}  // namespace webslam
