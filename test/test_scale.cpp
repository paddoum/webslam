// Verifies the metric scale estimator + camera-IMU auto-calibration. We
// synthesize a motion, derive a clean "IMU" as trueScale * acceleration, then
// feed it expressed in a deliberately ROTATED device frame. The estimator must
// discover the right device->camera alignment and still recover trueScale.
#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>

#include "../src/scale.h"

using namespace webslam;
using Eigen::Matrix3d;
using Eigen::Vector3d;

int main() {
  const double trueScale = 0.05;
  const double dt = 1.0 / 30.0;
  const int N = 150;

  auto pos = [](double t) {
    return Vector3d(std::sin(2.0 * t), 0.4 * std::sin(3.0 * t + 1.0), 0.3 * std::cos(2.5 * t));
  };
  auto acc = [](double t) {
    return Vector3d(-4.0 * std::sin(2.0 * t), -3.6 * std::sin(3.0 * t + 1.0),
                    -1.875 * std::cos(2.5 * t));
  };
  unsigned s = 99;
  auto noise = [&](double amp) { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return ((s % 10000) / 10000.0 - 0.5) * amp; };

  // A fixed but non-trivial device->camera rotation the estimator must discover
  // (90° about X — a signed axis permutation, like a real phone's extrinsic).
  Matrix3d Rcam_dev;
  Rcam_dev << 1, 0, 0, 0, 0, -1, 0, 1, 0;  // maps device axes into camera axes
  Matrix3d Rdev_cam = Rcam_dev.transpose();

  auto runCase = [&](double posNoise, double accNoise, double tol, const char* label) {
    MetricScaleEstimator est;
    for (int i = 0; i < N; ++i) {
      double t = i * dt;
      Vector3d p = pos(t) + Vector3d(noise(posNoise), noise(posNoise), noise(posNoise));
      // World-frame metric accel, then expressed in the device frame (Rwc = I
      // here, so world == camera). a_device = Rdev_cam * a_camera.
      Vector3d aWorld = trueScale * acc(t) + Vector3d(noise(accNoise), noise(accNoise), noise(accNoise));
      Vector3d aDevice = Rdev_cam * aWorld;
      est.addSample(t, p, Matrix3d::Identity(), aDevice);
    }
    ScaleResult r = est.result();
    printf("%-7s scale=%.4f (true %.4f) conf=%.3f axis=%d valid=%d\n",
           label, r.scale, trueScale, r.confidence, r.axis, r.valid);
    return r.valid && std::abs(r.scale - trueScale) / trueScale < tol;
  };

  bool ok = true;
  ok &= runCase(0.0, 0.0, 0.05, "clean");
  ok &= runCase(0.004, 0.02, 0.15, "noisy");
  printf("%s\n", ok ? "PASS (metric scale + calibration)" : "FAIL (metric scale + calibration)");
  return ok ? 0 : 1;
}
