// Integration test for M3 + M4: run a synthetic camera trajectory through the
// SLAM map. Frames are generated once, then replayed through the system twice —
// with local bundle adjustment OFF and ON — to show BA reduces trajectory drift.
#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <vector>

#include "../src/map.h"

using namespace webslam;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

int main() {
  Intrinsics K{420, 420, 160, 120};
  const int W = 320, H = 240;
  const int frames = 41;

  // World: 240 points with unique descriptors.
  struct WP { Vector3d X; Descriptor d; };
  std::vector<WP> world;
  uint64_t s = 20240611;
  auto rnd = [&]() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return ((s >> 11) % 1000000) / 1000000.0; };
  for (int i = 0; i < 240; ++i) {
    WP w;
    w.X = Vector3d((rnd() - 0.5) * 8, (rnd() - 0.5) * 6, 4 + rnd() * 10);
    for (int k = 0; k < 8; ++k) w.d[k] = (uint32_t)(rnd() * 4294967296.0);
    world.push_back(w);
  }

  auto gtPose = [&](int f) {
    const double u = f / 40.0;
    const double yaw = 0.5 * std::sin(u * 3.14159);
    Matrix3d Rwc;
    Rwc << std::cos(yaw), 0, std::sin(yaw), 0, 1, 0, -std::sin(yaw), 0, std::cos(yaw);
    Vector3d C(2.5 * u, 0.3 * std::sin(u * 6.28), 1.5 * u);
    Vector3d t = -Rwc * C;  // concrete Vector3d — never return an Eigen expression
    return std::make_pair(Rwc, t);
  };

  // Generate all frames up front (so both runs see identical input).
  std::vector<OrbFeatures> feats(frames);
  std::vector<Vector3d> gtCenters(frames);
  for (int f = 0; f < frames; ++f) {
    auto [Rwc, t] = gtPose(f);
    gtCenters[f] = -Rwc.transpose() * t;
    OrbFeatures& feat = feats[f];
    for (const auto& w : world) {
      Vector3d Xc = Rwc * w.X + t;
      if (Xc.z() < 0.5) continue;
      double px = K.fx * Xc.x() / Xc.z() + K.cx + (rnd() - 0.5) * 0.6;
      double py = K.fy * Xc.y() / Xc.z() + K.cy + (rnd() - 0.5) * 0.6;
      if (px < 0 || py < 0 || px >= W || py >= H) continue;
      feat.x.push_back((float)px);
      feat.y.push_back((float)py);
      feat.angle.push_back(0.0f);
      feat.desc.push_back(w.d);
    }
  }

  // Run the SLAM map over the prepared frames; return RMS trajectory error
  // after similarity alignment (monocular gauge freedom).
  auto run = [&](bool ba, int& tracked, int& pts, int& kfs) {
    SlamMap slam(K);
    slam.baEnabled = ba;
    std::vector<Vector3d> est, gt;
    tracked = 0;
    for (int f = 0; f < frames; ++f) {
      bool ok = slam.process(feats[f]);
      if (ok && slam.state() == SlamMap::State::kTracking) {
        ++tracked;
        gt.push_back(gtCenters[f]);
        est.push_back(slam.cameraCenter());
      }
    }
    pts = slam.numPoints();
    kfs = slam.numKeyframes();
    if (est.size() < 10) return 1e9;
    Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, est.size()), dst(3, gt.size());
    for (size_t i = 0; i < est.size(); ++i) { src.col(i) = est[i]; dst.col(i) = gt[i]; }
    Eigen::Matrix4d T = Eigen::umeyama(src, dst, true);
    double sse = 0, len = 0;
    for (size_t i = 0; i < est.size(); ++i) {
      Eigen::Vector4d p(src(0, i), src(1, i), src(2, i), 1.0);
      sse += ((T * p).head<3>() - dst.col(i)).squaredNorm();
    }
    for (size_t i = 1; i < gt.size(); ++i) len += (gt[i] - gt[i - 1]).norm();
    return std::sqrt(sse / est.size()) / len;  // relative RMS
  };

  int tA, pA, kA, tB, pB, kB;
  double relOff = run(false, tA, pA, kA);
  double relOn = run(true, tB, pB, kB);
  printf("BA off: tracked %d/%d, %d points, %d kf, trajectory RMS %.2f%%\n", tA, frames, pA, kA, relOff * 100);
  printf("BA on : tracked %d/%d, %d points, %d kf, trajectory RMS %.2f%%\n", tB, frames, pB, kB, relOn * 100);
  printf("BA improvement: %.2f%% -> %.2f%%  (%.0f%% lower error)\n", relOff * 100, relOn * 100,
         (1 - relOn / relOff) * 100);

  const bool pass = tB >= frames - 3 && pB > 80 && relOn < 0.05 && relOn <= relOff;
  printf("%s\n", pass ? "PASS (map + bundle adjustment)" : "FAIL (map + bundle adjustment)");
  return pass ? 0 : 1;
}
