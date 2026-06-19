// Native unit test for the FAST corner detector.
// Builds a synthetic frame with a white rectangle on a dark background and
// asserts that corners are detected near the rectangle's four corners.
// This verifies the same C++ code that gets compiled to WASM.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "../src/slam_engine.h"

using webslam::SlamEngine;

int main() {
  const int W = 320, H = 240;
  SlamEngine engine(W, H, 800);

  // Write a synthetic RGBA frame directly into the engine's input buffer:
  // dark background (40) with a bright rectangle (230) from (100,80)-(220,170).
  auto* buf = reinterpret_cast<unsigned char*>(engine.inputBufferPtr());
  const int rx0 = 100, ry0 = 80, rx1 = 220, ry1 = 170;
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const bool inside = (x >= rx0 && x < rx1 && y >= ry0 && y < ry1);
      const unsigned char v = inside ? 230 : 40;
      const int i = (y * W + x) * 4;
      buf[i + 0] = v; buf[i + 1] = v; buf[i + 2] = v; buf[i + 3] = 255;
    }
  }

  const int n = engine.processFrame(W, H, 28);
  const auto& kp = engine.keypointsFlat();
  printf("detected %d keypoints\n", n);

  // The four rectangle corners we expect to find a feature near.
  const float corners[4][2] = {
      {(float)rx0, (float)ry0}, {(float)rx1 - 1, (float)ry0},
      {(float)rx0, (float)ry1 - 1}, {(float)rx1 - 1, (float)ry1 - 1}};

  int found = 0;
  for (auto& c : corners) {
    float best = 1e9f;
    for (size_t i = 0; i < kp.size(); i += 3) {
      const float dx = kp[i] - c[0], dy = kp[i + 1] - c[1];
      best = std::min(best, std::sqrt(dx * dx + dy * dy));
    }
    const bool ok = best <= 4.0f;  // within 4 px
    printf("  corner (%.0f,%.0f): nearest feature %.1f px  %s\n", c[0], c[1],
           best, ok ? "OK" : "MISS");
    found += ok ? 1 : 0;
  }

  if (n > 0 && found == 4) {
    printf("PASS: all 4 rectangle corners detected\n");
    return 0;
  }
  printf("FAIL: only %d/4 corners detected\n", found);
  return 1;
}
