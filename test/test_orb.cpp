// Verifies ORB descriptor extraction + Hamming matching: build a textured
// image, shift it by a known (dx,dy), and confirm the matches reproduce that
// displacement for the large majority of correspondences.
#include <cmath>
#include <cstdio>
#include <vector>

#include "../src/orb.h"
#include "../src/slam_engine.h"

using namespace webslam;

int main() {
  const int W = 320, H = 240;
  const int dx = 7, dy = 4;  // ground-truth shift from A to B

  // Build a richly-textured grayscale image (what a real camera sees): a smooth
  // non-repeating background plus ~400 small random dots, so every corner's
  // local neighborhood is distinctive. Rendering is offset by (ox,oy) to
  // simulate a pure image-plane shift.
  auto makeImage = [&](int ox, int oy) {
    std::vector<uint8_t> g(W * H);
    // Fixed random dot field (generated in a virtual canvas, then sampled with offset).
    struct Dot { int x, y, r, v; };
    std::vector<Dot> dots;
    uint64_t s = 1234567;
    auto rnd = [&]() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return (s % 100000) / 100000.0; };
    for (int i = 0; i < 400; ++i)
      dots.push_back({(int)(rnd() * (W + 40)) - 20, (int)(rnd() * (H + 40)) - 20,
                      1 + (int)(rnd() * 2), 30 + (int)(rnd() * 200)});
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const double wx = x - ox, wy = y - oy;  // so a feature moves by +(ox,oy) from A to B
        double v = 128 + 30 * std::sin(wx * 0.013) + 24 * std::cos(wy * 0.011) +
                   18 * std::sin((wx + wy) * 0.007);
        for (const auto& d : dots) {
          const double dd = (wx - d.x) * (wx - d.x) + (wy - d.y) * (wy - d.y);
          if (dd <= (double)(d.r * d.r)) v = d.v;
        }
        g[y * W + x] = (uint8_t)std::max(0.0, std::min(255.0, v));
      }
    }
    return g;
  };

  auto grayA = makeImage(0, 0);
  auto grayB = makeImage(dx, dy);  // same scene, shifted

  auto kpsA = detectFAST(grayA.data(), W, H, 20, 800);
  auto kpsB = detectFAST(grayB.data(), W, H, 20, 800);
  printf("keypoints: A=%zu B=%zu\n", kpsA.size(), kpsB.size());

  OrbFeatures fa, fb;
  computeOrb(grayA.data(), W, H, kpsA, 18, fa);
  computeOrb(grayB.data(), W, H, kpsB, 18, fb);

  auto matches = matchHamming(fa, fb, 64, 0.8f, true);
  printf("matches: %zu\n", matches.size());

  int correct = 0;
  for (auto& m : matches) {
    const float ddx = fb.x[m.b] - fa.x[m.a];
    const float ddy = fb.y[m.b] - fa.y[m.a];
    if (std::abs(ddx - dx) < 1.5f && std::abs(ddy - dy) < 1.5f) ++correct;
  }
  const double frac = matches.empty() ? 0.0 : (double)correct / matches.size();
  printf("displacement-consistent matches: %d / %zu (%.0f%%)\n",
         correct, matches.size(), frac * 100);

  const bool pass = matches.size() >= 40 && frac >= 0.8;
  printf("%s\n", pass ? "PASS (ORB matching)" : "FAIL (ORB matching)");
  return pass ? 0 : 1;
}
