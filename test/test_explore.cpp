// Verifies the sliding map: a camera translates down a long corridor of points
// with a SMALL map budget. The init region leaves view long before the end, so
// staying tracked at the far end is only possible if new area is continuously
// mapped while old points are culled — and the map count must stay bounded.
#include <cmath>
#include <cstdio>
#include <vector>

#include "../src/map.h"

using namespace webslam;
using Eigen::Matrix3d;
using Eigen::Vector3d;

int main() {
  Intrinsics K{420, 420, 160, 120};
  const int W = 320, H = 240, frames = 90;

  // A long corridor of uniquely-described points along +X.
  struct WP { Vector3d X; Descriptor d; };
  std::vector<WP> world;
  uint64_t s = 12345;
  auto rnd = [&]() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return ((s >> 11) % 1000000) / 1000000.0; };
  for (int i = 0; i < 900; ++i) {
    WP w; w.X = Vector3d(-2 + rnd() * 24, (rnd() - 0.5) * 4, 4 + rnd() * 3);  // x in [-2,22]
    for (int k = 0; k < 8; ++k) w.d[k] = (uint32_t)(rnd() * 4294967296.0);
    world.push_back(w);
  }
  // Camera looks along +Z and slides along +X from 0 to 16.
  auto camPose = [&](int f) {
    double cx = 16.0 * f / (frames - 1);
    Matrix3d R = Matrix3d::Identity();
    Vector3d t(-cx, 0, 0);  // R=I so Xc = X - C, C=(cx,0,0)
    return std::make_pair(R, t);
  };

  SlamMap slam(K);
  slam.maxMapPoints = 200;  // small budget -> forces culling during exploration

  int tracked = 0, trackedLate = 0, maxPts = 0;
  double startCamX = 0, endCamX = 0;
  for (int f = 0; f < frames; ++f) {
    auto [R, t] = camPose(f);
    OrbFeatures ft;
    for (const auto& w : world) {
      Vector3d Xc = R * w.X + t;
      if (Xc.z() < 0.5) continue;
      double px = K.fx * Xc.x() / Xc.z() + K.cx + (rnd() - 0.5) * 0.5;
      double py = K.fy * Xc.y() / Xc.z() + K.cy + (rnd() - 0.5) * 0.5;
      if (px < 0 || py < 0 || px >= W || py >= H) continue;
      ft.x.push_back((float)px); ft.y.push_back((float)py); ft.angle.push_back(0); ft.desc.push_back(w.d);
    }
    bool ok = slam.process(ft);
    maxPts = std::max(maxPts, slam.numPoints());
    if (ok && slam.state() == SlamMap::State::kTracking) {
      ++tracked;
      if (f >= frames - 12) ++trackedLate;  // far from the init region
    }
  }

  printf("tracked %d/%d   tracked in final 12 frames: %d/12\n", tracked, frames, trackedLate);
  printf("max map points: %d (budget 200)   keyframes: %d\n", maxPts, slam.numKeyframes());

  const bool pass = tracked >= frames * 0.8 && trackedLate >= 9 && maxPts <= 210;
  printf("%s\n", pass ? "PASS (sliding map / exploration)" : "FAIL (sliding map / exploration)");
  return pass ? 0 : 1;
}
