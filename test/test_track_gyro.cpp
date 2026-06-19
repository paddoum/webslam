// Verifies the gyro rotation PRIOR (M13). During a fast, non-constant rotation
// the constant-velocity predictor lags, so map points project away from where
// they land; matches fall outside the search window and PnP keeps fewer inliers
// (and tracking eventually drops). Feeding the true camera-frame angular
// velocity via setGyroDelta() centres the prediction correctly, so matches land
// and inliers stay high.
//
// We run the SAME fast-spin sequence twice — without and with the gyro prior —
// and compare (a) frames tracked and (b) total PnP inliers across the spin.
//
// HONEST NOTE: on CLEAN synthetic features the existing adaptive search radius
// already compensates for a lagging prediction, so the measurable gain here is
// small (this is a no-regression + small-gain guard, not proof of the on-device
// benefit). The gyro prior matters most on real data — motion blur, sparse/
// aliased features, and recovery after loss — which simulation doesn't model.
#include <cmath>
#include <cstdio>
#include <vector>

#include "../src/imu_preint.h"  // logSO3
#include "../src/map.h"

using namespace webslam;
using Eigen::Matrix3d;
using Eigen::Vector3d;

int main() {
  Intrinsics K{420, 420, 160, 120};
  const int W = 320, H = 240, frames = 60;
  const int spinStart = 20;

  struct WP { Vector3d X; Descriptor d; };
  std::vector<WP> world;
  uint64_t s = 7;
  auto rnd = [&]() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return ((s >> 11) % 1000000) / 1000000.0; };
  for (int i = 0; i < 700; ++i) {
    WP w; w.X = Vector3d((rnd() - 0.5) * 26, (rnd() - 0.5) * 6, 4 + rnd() * 8);
    for (int k = 0; k < 8; ++k) w.d[k] = (uint32_t)(rnd() * 4294967296.0);
    world.push_back(w);
  }

  // Gentle for the first 20 frames, then a fast yaw oscillation whose RATE
  // changes every frame (constant-velocity can't anticipate it).
  auto yawOf = [&](int f) {
    if (f < spinStart) return 0.02 * f;
    const double u = f - spinStart;
    return 0.4 + 0.45 * std::sin(0.7 * u);     // fast, non-constant yaw (~18°/frame peak)
  };
  auto gtPose = [&](int f) {
    const double yaw = yawOf(f);
    Matrix3d Rwc; Rwc << std::cos(yaw), 0, std::sin(yaw), 0, 1, 0, -std::sin(yaw), 0, std::cos(yaw);
    Vector3d C(0.3 * (f / 59.0), 0.05 * std::sin(f * 0.3), 0.2 * (f / 59.0));
    return std::make_pair(Rwc, Vector3d(-Rwc * C));
  };
  auto frameFeatures = [&](int f) {
    auto [Rwc, t] = gtPose(f);
    OrbFeatures ft;
    for (const auto& w : world) {
      Vector3d Xc = Rwc * w.X + t;
      if (Xc.z() < 0.5) continue;
      double px = K.fx * Xc.x() / Xc.z() + K.cx + (rnd() - 0.5) * 0.6;
      double py = K.fy * Xc.y() / Xc.z() + K.cy + (rnd() - 0.5) * 0.6;
      if (px < 0 || py < 0 || px >= W || py >= H) continue;
      ft.x.push_back((float)px); ft.y.push_back((float)py); ft.angle.push_back(0); ft.desc.push_back(w.d);
    }
    return ft;
  };
  auto gyroOf = [&](int f) {  // ω = -logSO3(Rwc(f)·Rwc(f-1)^T) (dt=1), matches setGyroDelta
    auto [Rc, tc] = gtPose(f);
    auto [Rp, tp] = gtPose(f - 1);
    return Vector3d(-logSO3(Rc * Rp.transpose()));
  };

  auto run = [&](bool useGyro, int& tracked, long& inliers) {
    SlamMap slam(K);
    tracked = 0; inliers = 0;
    for (int f = 0; f < frames; ++f) {
      if (useGyro && f > 0) { Vector3d w = gyroOf(f); slam.setGyroDelta(w, 1.0); }
      slam.process(frameFeatures(f));
      if (f >= spinStart && slam.tracked()) { ++tracked; inliers += slam.lastInliers(); }
    }
  };

  int cvT, gyT; long cvI, gyI;
  run(false, cvT, cvI);
  run(true,  gyT, gyI);
  const int spin = frames - spinStart;
  printf("spin [%d,%d): tracked  CV=%d/%d  gyro=%d/%d\n", spinStart, frames, cvT, spin, gyT, spin);
  printf("              inliers  CV=%ld      gyro=%ld\n", cvI, gyI);

  // No-regression: the gyro prior must never track fewer frames and never lose
  // inliers vs the constant-velocity baseline (with a correct gyro it can only
  // help the prediction). A clear win requires real data — see the note above.
  const bool pass = gyT >= cvT && gyI >= cvI;
  printf("%s\n", pass ? "PASS (gyro prior >= constant-velocity; no regression)" : "FAIL (gyro prior)");
  return pass ? 0 : 1;
}
