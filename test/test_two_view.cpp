// Verifies the two-view geometry core: project a known 3D cloud into two
// cameras with a known relative pose, then check estimateRelativePose recovers
// that pose (rotation to <1 deg, translation direction to <2 deg).
#include <cmath>
#include <cstdio>

#include "../src/two_view.h"

using namespace webslam;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

static double angleBetween(const Vector3d& a, const Vector3d& b) {
  double c = a.normalized().dot(b.normalized());
  c = std::max(-1.0, std::min(1.0, c));
  return std::acos(c) * 180.0 / M_PI;
}

static double rotationError(const Matrix3d& A, const Matrix3d& B) {
  Matrix3d d = A.transpose() * B;
  double tr = (d.trace() - 1.0) / 2.0;
  tr = std::max(-1.0, std::min(1.0, tr));
  return std::acos(tr) * 180.0 / M_PI;
}

int main() {
  Intrinsics K{500, 500, 320, 240};

  // Ground-truth relative pose: 10 deg about Y, then translate.
  const double a = 10.0 * M_PI / 180.0;
  Matrix3d Rgt;
  Rgt << std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a);
  Vector3d tgt(0.4, 0.05, 0.12);  // metric translation (scale will be lost)

  // Deterministic 3D cloud in front of camera 1.
  std::vector<Match2D> matches;
  unsigned s = 7;
  auto rnd = [&]() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return (s % 10000) / 10000.0; };
  for (int i = 0; i < 120; ++i) {
    Vector3d X((rnd() - 0.5) * 4.0, (rnd() - 0.5) * 3.0, 2.0 + rnd() * 6.0);
    Vector3d Xc2 = Rgt * X + tgt;
    if (X.z() <= 0 || Xc2.z() <= 0) continue;
    Vector2d p1(K.fx * X.x() / X.z() + K.cx, K.fy * X.y() / X.z() + K.cy);
    Vector2d p2(K.fx * Xc2.x() / Xc2.z() + K.cx, K.fy * Xc2.y() / Xc2.z() + K.cy);
    matches.push_back({p1, p2});
  }
  printf("synthetic correspondences: %zu\n", matches.size());

  RelativePose rp = estimateRelativePose(matches, K);
  if (!rp.ok) { printf("FAIL: no pose recovered\n"); return 1; }

  const double rErr = rotationError(Rgt, rp.R);
  const double tErr = angleBetween(tgt, rp.t);
  printf("inliers: %d / %zu\n", rp.inliers, matches.size());
  printf("rotation error:    %.3f deg\n", rErr);
  printf("translation error: %.3f deg (direction)\n", tErr);

  const bool passClean = rp.inliers >= (int)matches.size() * 0.9 && rErr < 1.0 && tErr < 2.0;
  printf("%s\n", passClean ? "PASS (clean)" : "FAIL (clean)");

  // --- Noisy + outlier scenario: 0.5 px noise, 25% gross outliers. ---
  std::vector<Match2D> dirty = matches;
  int nOut = 0;
  for (size_t i = 0; i < dirty.size(); ++i) {
    dirty[i].a += Vector2d((rnd() - 0.5), (rnd() - 0.5));   // ~+-0.5 px
    dirty[i].b += Vector2d((rnd() - 0.5), (rnd() - 0.5));
    if (rnd() < 0.25) {                                     // gross outlier in b
      dirty[i].b = Vector2d(rnd() * 640, rnd() * 480);
      ++nOut;
    }
  }
  RelativePose rp2 = estimateRelativePose(dirty, K, 2.0, 500);
  const double rErr2 = rotationError(Rgt, rp2.R);
  const double tErr2 = angleBetween(tgt, rp2.t);
  printf("noisy: %d outliers injected, inliers kept %d / %zu, rotErr %.2f deg, tErr %.2f deg\n",
         nOut, rp2.inliers, dirty.size(), rErr2, tErr2);
  const bool passNoisy = rp2.ok && rErr2 < 2.0 && tErr2 < 6.0;
  printf("%s\n", passNoisy ? "PASS (noisy+outliers)" : "FAIL (noisy+outliers)");

  return (passClean && passNoisy) ? 0 : 1;
}
