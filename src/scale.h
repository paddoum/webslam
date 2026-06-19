#pragma once
#include <Eigen/Dense>
#include <deque>
#include <vector>

namespace webslam {

struct ScaleResult {
  double scale = 0;       // meters per visual unit (raw estimate when computable)
  double confidence = 0;  // [0,1] — alignment of the two velocity signals
  int samples = 0;        // window samples used
  int axis = -1;          // index of the chosen device->camera alignment (calibration)
  bool valid = false;     // confident + enough motion
};

// Metric scale estimator with camera-IMU rotation auto-calibration.
//
// Monocular SLAM recovers motion up to scale; the accelerometer measures m/s^2,
// but in the DEVICE frame, related to the camera frame by an unknown rigid
// rotation. Rather than guess that rotation, we try the 24 signed axis
// alignments (the camera-IMU extrinsic of a phone is essentially one of them)
// and keep the one whose acceleration best correlates with the visual
// trajectory. For the winning alignment we report the metric scale (TLS slope
// of integrated IMU velocity vs differentiated visual velocity).
class MetricScaleEstimator {
 public:
  MetricScaleEstimator();
  void reset();

  // Add one synchronized sample: visual camera position (world units), the
  // world->camera rotation Rwc, and the RAW device-frame linear acceleration
  // (m/s^2, gravity removed). t in seconds.
  void addSample(double t, const Eigen::Vector3d& visualPos, const Eigen::Matrix3d& Rwc,
                 const Eigen::Vector3d& aDevice);

  ScaleResult result() const { return last_; }

  size_t windowSamples = 90;
  double minConfidence = 0.3;
  double minVisualAccel = 1e-5;

 private:
  // Score one device->camera candidate rotation over the window.
  bool scoreCandidate(const Eigen::Matrix3d& Rci, double& scale, double& confidence) const;

  struct Sample { double t; Eigen::Vector3d p; Eigen::Matrix3d Rwc; Eigen::Vector3d aDev; };
  std::deque<Sample> window_;
  std::vector<Eigen::Matrix3d> candidates_;  // 24 signed axis alignments
  ScaleResult last_;
};

}  // namespace webslam
