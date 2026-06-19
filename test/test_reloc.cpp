// Verifies relocalization: build a map, force tracking loss by feeding blank
// frames, then resume real frames and confirm the system RECOVERS (returns to
// tracking) without resetting the map — i.e. keyframes/points are retained.
#include <cmath>
#include <cstdio>
#include <vector>

#include "../src/map.h"

using namespace webslam;
using Eigen::Matrix3d;
using Eigen::Vector3d;

int main() {
  Intrinsics K{420, 420, 160, 120};
  const int W = 320, H = 240, frames = 50;

  struct WP { Vector3d X; Descriptor d; };
  std::vector<WP> world;
  uint64_t s = 7;
  auto rnd = [&]() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return ((s >> 11) % 1000000) / 1000000.0; };
  for (int i = 0; i < 240; ++i) {
    WP w; w.X = Vector3d((rnd() - 0.5) * 8, (rnd() - 0.5) * 6, 4 + rnd() * 10);
    for (int k = 0; k < 8; ++k) w.d[k] = (uint32_t)(rnd() * 4294967296.0);
    world.push_back(w);
  }
  auto gtPose = [&](int f) {
    const double u = f / 49.0;
    const double yaw = 0.5 * std::sin(u * 3.14159);
    Matrix3d Rwc; Rwc << std::cos(yaw), 0, std::sin(yaw), 0, 1, 0, -std::sin(yaw), 0, std::cos(yaw);
    Vector3d C(2.5 * u, 0.3 * std::sin(u * 6.28), 1.5 * u);
    Vector3d t = -Rwc * C;
    return std::make_pair(Rwc, t);
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

  SlamMap slam(K);
  bool everLost = false, recovered = false;
  int kfBeforeGap = 0, ptsBeforeGap = 0;
  const int gapStart = 25, gapLen = 4;  // blank frames -> forced loss

  for (int f = 0; f < frames; ++f) {
    OrbFeatures ft;
    if (f >= gapStart && f < gapStart + gapLen) {
      // blank frame (no matchable features) -> tracking must drop
      if (f == gapStart) { kfBeforeGap = slam.numKeyframes(); ptsBeforeGap = slam.numPoints(); }
    } else {
      ft = frameFeatures(f);
    }
    slam.process(ft);
    if (slam.state() == SlamMap::State::kLost) everLost = true;
    if (everLost && f > gapStart + gapLen && slam.state() == SlamMap::State::kTracking) recovered = true;
  }

  printf("lost during gap: %d   recovered after gap: %d\n", everLost, recovered);
  printf("keyframes %d->%d, points %d->%d (retained across loss, NOT reset)\n",
         kfBeforeGap, slam.numKeyframes(), ptsBeforeGap, slam.numPoints());

  const bool pass = everLost && recovered &&
                    slam.numKeyframes() >= kfBeforeGap && slam.numPoints() >= ptsBeforeGap;
  printf("%s\n", pass ? "PASS (relocalization)" : "FAIL (relocalization)");
  return pass ? 0 : 1;
}
