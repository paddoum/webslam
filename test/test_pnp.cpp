// Verifies the PnP solver: project a known 3D cloud with a known camera pose,
// then check solvePnP recovers that pose — from scratch (no prior) and with
// 30% gross 2D outliers.
#include <cmath>
#include <cstdio>

#include "../src/pnp.h"

using namespace webslam;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

static double rotErr(const Matrix3d& A, const Matrix3d& B) {
  double tr = ((A.transpose() * B).trace() - 1.0) / 2.0;
  tr = std::max(-1.0, std::min(1.0, tr));
  return std::acos(tr) * 180.0 / M_PI;
}

int main() {
  Intrinsics K{500, 500, 320, 240};

  // Ground-truth pose (world -> camera): 25 deg yaw + translation.
  const double a = 25.0 * M_PI / 180.0;
  Matrix3d Rgt;
  Rgt << std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a);
  Vector3d tgt(0.5, -0.2, 0.3);

  std::vector<Vector3d> X;
  std::vector<Vector2d> uv;
  unsigned s = 42;
  auto rnd = [&]() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return (s % 10000) / 10000.0; };
  for (int i = 0; i < 100; ++i) {
    Vector3d Xw((rnd() - 0.5) * 4, (rnd() - 0.5) * 3, 2 + rnd() * 6);
    Vector3d Xc = Rgt * Xw + tgt;
    if (Xc.z() <= 0.1) continue;
    X.push_back(Xw);
    uv.push_back(Vector2d(K.fx * Xc.x() / Xc.z() + K.cx, K.fy * Xc.y() / Xc.z() + K.cy));
  }
  printf("points: %zu\n", X.size());

  // 1) Clean, no prior.
  PnPResult r1 = solvePnP(X, uv, K);
  printf("clean:  ok=%d inliers=%d/%zu rotErr=%.3f deg  tErr=%.4f\n", r1.ok, r1.inliers,
         X.size(), rotErr(Rgt, r1.R), (r1.t - tgt).norm());
  const bool pass1 = r1.ok && rotErr(Rgt, r1.R) < 0.5 && (r1.t - tgt).norm() < 0.02;

  // 2) 30% gross 2D outliers.
  auto uv2 = uv;
  int nOut = 0;
  for (size_t i = 0; i < uv2.size(); ++i)
    if (rnd() < 0.30) { uv2[i] = Vector2d(rnd() * 640, rnd() * 480); ++nOut; }
  PnPResult r2 = solvePnP(X, uv2, K, 3.0, 300);
  printf("noisy:  %d outliers, ok=%d inliers=%d/%zu rotErr=%.3f deg tErr=%.4f\n", nOut, r2.ok,
         r2.inliers, uv2.size(), rotErr(Rgt, r2.R), (r2.t - tgt).norm());
  const bool pass2 = r2.ok && rotErr(Rgt, r2.R) < 1.0 && (r2.t - tgt).norm() < 0.05;

  const bool pass = pass1 && pass2;
  printf("%s\n", pass ? "PASS (PnP)" : "FAIL (PnP)");
  return pass ? 0 : 1;
}
