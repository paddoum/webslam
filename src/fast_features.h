#pragma once
#include <cstdint>
#include <vector>

namespace webslam {

// A detected 2D feature: pixel location + corner strength.
struct Keypoint {
  float x;
  float y;
  float score;
};

// FAST-9 corner detection on a grayscale image. Returns up to `maxFeatures`
// strongest corners after 3x3 non-maximum suppression.
std::vector<Keypoint> detectFAST(const std::uint8_t* gray, int width, int height,
                                 int threshold, int maxFeatures);

}  // namespace webslam
