#include "orb.h"

#include <cmath>

namespace webslam {

namespace {

constexpr int kPatchRadius = 15;     // orientation patch radius
constexpr int kBriefRadius = 13;     // BRIEF sampling radius
constexpr int kNumBits = 256;        // descriptor length

// A BRIEF test: two sample offsets (ax,ay) vs (bx,by) within the patch.
struct BriefPair { float ax, ay, bx, by; };

// Generate a fixed, deterministic BRIEF sampling pattern (Gaussian-distributed
// point pairs). Built once; identical across frames so descriptors compare.
const std::array<BriefPair, kNumBits>& briefPattern() {
  static std::array<BriefPair, kNumBits> pattern = [] {
    std::array<BriefPair, kNumBits> p{};
    uint64_t s = 0xB5297A4D;  // fixed seed
    auto gauss = [&]() {
      // crude normal via sum of uniforms, scaled to the sampling radius.
      double u = 0;
      for (int k = 0; k < 6; ++k) { s ^= s << 13; s ^= s >> 7; s ^= s << 17; u += (s % 10000) / 10000.0; }
      return (u - 3.0) / 3.0 * kBriefRadius;  // ~[-r, r]
    };
    for (int i = 0; i < kNumBits; ++i) p[i] = {(float)gauss(), (float)gauss(), (float)gauss(), (float)gauss()};
    return p;
  }();
  return pattern;
}

// 3x3 box-averaged intensity sample (clamped), for BRIEF stability.
inline int sampleSmooth(const std::uint8_t* gray, int w, int h, int x, int y) {
  int sum = 0, cnt = 0;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      int xx = x + dx, yy = y + dy;
      if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
      sum += gray[yy * w + xx];
      ++cnt;
    }
  }
  return cnt ? sum / cnt : 0;
}

// Intensity-centroid orientation (Rosten/Rublee).
float orientation(const std::uint8_t* gray, int w, int h, int cx, int cy) {
  double m01 = 0, m10 = 0;
  const int r2 = kPatchRadius * kPatchRadius;
  for (int dy = -kPatchRadius; dy <= kPatchRadius; ++dy) {
    for (int dx = -kPatchRadius; dx <= kPatchRadius; ++dx) {
      if (dx * dx + dy * dy > r2) continue;
      int xx = cx + dx, yy = cy + dy;
      if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
      const int v = gray[yy * w + xx];
      m10 += dx * v;
      m01 += dy * v;
    }
  }
  return std::atan2(m01, m10);
}

}  // namespace

void computeOrb(const std::uint8_t* gray, int width, int height,
                const std::vector<Keypoint>& kps, int border, OrbFeatures& out) {
  out.x.clear(); out.y.clear(); out.angle.clear(); out.desc.clear();
  const auto& pattern = briefPattern();

  for (const auto& kp : kps) {
    const int cx = (int)std::lround(kp.x);
    const int cy = (int)std::lround(kp.y);
    if (cx < border || cy < border || cx >= width - border || cy >= height - border) continue;

    const float theta = orientation(gray, width, height, cx, cy);
    const float ct = std::cos(theta), st = std::sin(theta);

    Descriptor d{};
    for (int i = 0; i < kNumBits; ++i) {
      const BriefPair& bp = pattern[i];
      // Rotate the sample pair by the keypoint orientation (steered BRIEF).
      const int ax = cx + (int)std::lround(bp.ax * ct - bp.ay * st);
      const int ay = cy + (int)std::lround(bp.ax * st + bp.ay * ct);
      const int bx = cx + (int)std::lround(bp.bx * ct - bp.by * st);
      const int by = cy + (int)std::lround(bp.bx * st + bp.by * ct);
      const int va = sampleSmooth(gray, width, height, ax, ay);
      const int vb = sampleSmooth(gray, width, height, bx, by);
      if (va < vb) d[i >> 5] |= (1u << (i & 31));
    }

    out.x.push_back(kp.x);
    out.y.push_back(kp.y);
    out.angle.push_back(theta);
    out.desc.push_back(d);
  }
}

namespace {
inline int hamming(const Descriptor& a, const Descriptor& b) {
  int d = 0;
  for (int i = 0; i < 8; ++i) d += __builtin_popcount(a[i] ^ b[i]);
  return d;
}
}  // namespace

int hammingDistance(const Descriptor& a, const Descriptor& b) { return hamming(a, b); }

std::vector<DMatch> matchHamming(const OrbFeatures& A, const OrbFeatures& B,
                                 int maxDist, float ratio, bool crossCheck) {
  std::vector<DMatch> matches;
  const int na = (int)A.size(), nb = (int)B.size();
  if (na == 0 || nb == 0) return matches;

  // For cross-check: best B->A index for each b.
  std::vector<int> bestBtoA;
  if (crossCheck) {
    bestBtoA.assign(nb, -1);
    for (int j = 0; j < nb; ++j) {
      int best = 1e9, bi = -1;
      for (int i = 0; i < na; ++i) {
        int dd = hamming(B.desc[j], A.desc[i]);
        if (dd < best) { best = dd; bi = i; }
      }
      bestBtoA[j] = bi;
    }
  }

  for (int i = 0; i < na; ++i) {
    int best = 1e9, second = 1e9, bj = -1;
    for (int j = 0; j < nb; ++j) {
      int dd = hamming(A.desc[i], B.desc[j]);
      if (dd < best) { second = best; best = dd; bj = j; }
      else if (dd < second) { second = dd; }
    }
    if (bj < 0 || best > maxDist) continue;
    if (second < 1e9 && best > ratio * second) continue;     // Lowe ratio test
    if (crossCheck && bestBtoA[bj] != i) continue;            // mutual best
    matches.push_back({i, bj, best});
  }
  return matches;
}

}  // namespace webslam
