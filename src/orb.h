#pragma once
#include <array>
#include <cstdint>
#include <vector>

#include "fast_features.h"  // for webslam::Keypoint

namespace webslam {

// 256-bit ORB descriptor packed into 8 x 32-bit words.
using Descriptor = std::array<std::uint32_t, 8>;

// 64-D float XFeat (learned) descriptor, L2-normalized. Used only by the
// optional XFeat relocalization channel (see SlamMap::relocalizeByXFeat).
using XDescriptor = std::array<float, 64>;

// Squared L2 distance between two XFeat descriptors. Both are unit-norm, so
// this equals 2 - 2*cosine; smaller = more similar. Used for mutual-NN + ratio.
inline float xfeatDist2(const XDescriptor& a, const XDescriptor& b) {
  float s = 0;
  for (int i = 0; i < 64; ++i) { const float d = a[i] - b[i]; s += d * d; }
  return s;
}

// A set of keypoints that survived descriptor extraction, with their
// orientation and binary descriptor.
struct OrbFeatures {
  std::vector<float> x, y;       // pixel coordinates
  std::vector<float> angle;      // orientation (radians)
  std::vector<Descriptor> desc;  // rotated-BRIEF descriptors
  size_t size() const { return x.size(); }
};

// Compute oriented FAST orientation + rotated BRIEF descriptors for the given
// keypoints on a grayscale image. Keypoints too close to the border (within
// `border` px) are dropped. This is the open analogue of the engine's GORB.
void computeOrb(const std::uint8_t* gray, int width, int height,
                const std::vector<Keypoint>& kps, int border, OrbFeatures& out);

// One descriptor match between feature set A and B.
struct DMatch {
  int a;      // index into A
  int b;      // index into B
  int dist;   // Hamming distance
};

// Hamming distance between two 256-bit descriptors.
int hammingDistance(const Descriptor& a, const Descriptor& b);

// Brute-force Hamming matcher with Lowe ratio test and optional cross-check.
std::vector<DMatch> matchHamming(const OrbFeatures& A, const OrbFeatures& B,
                                 int maxDist, float ratio, bool crossCheck);

}  // namespace webslam
