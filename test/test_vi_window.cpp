// Verifies the live VI-init driver (M12.4): stream high-rate IMU + up-to-scale
// visual poses through ViWindow exactly as the device does, and confirm it
// recovers the metric scale and gravity from the accumulated window. Also
// checks that a tracking loss breaks the window but keeps the last good result.
#include <cmath>
#include <cstdio>

#include "../src/imu_preint.h"
#include "../src/vi_window.h"

using namespace webslam;
using Eigen::Matrix3d;
using Eigen::Vector3d;

int main() {
  const Vector3d g(0, 0, -9.81);          // world-frame gravity
  const double trueScale = 0.5;           // metres per visual unit
  const Vector3d w0(0.3, -0.2, 0.35);     // constant body angular velocity (gyro)
  const Vector3d Amp(0.6, 0.4, 0.5), wf(1.6, 2.2, 1.9);  // metric trajectory

  auto P = [&](double t){ return Vector3d(Amp.x()*std::sin(wf.x()*t), Amp.y()*std::sin(wf.y()*t), Amp.z()*std::sin(wf.z()*t)); };
  auto A = [&](double t){ return Vector3d(-Amp.x()*wf.x()*wf.x()*std::sin(wf.x()*t), -Amp.y()*wf.y()*wf.y()*std::sin(wf.y()*t), -Amp.z()*wf.z()*wf.z()*std::sin(wf.z()*t)); };
  auto Rwb = [&](double t){ return expSO3(w0 * t); };  // body->world

  ViWindow vi;
  const double imuDt = 1.0/200, dur = 2.5;
  const int imuPerFrame = 7;             // ~28 Hz camera frames

  int n = 0;
  for (double t = 0; t < dur; t += imuDt, ++n) {
    // High-rate IMU sample in the (camera=body) frame: gyro and specific force
    // INCLUDING gravity: f_body = R_bw (a_world - g_world).
    ImuSample s;
    s.dt = imuDt;
    s.gyro = w0;
    s.acc = Rwb(t).transpose() * (A(t) - g);
    vi.addImu(s);
    // Offer a visual pose at the camera-frame cadence (up-to-scale positions).
    if (n % imuPerFrame == 0) {
      vi.onTrackedFrame(Rwb(t), P(t) / trueScale);
    }
  }

  printf("window kf=%d  ok=%d\n", vi.windowKeyframes(), vi.ok());
  printf("scale %.4f (true %.4f)  |g| %.3f  conf %.2f\n",
         vi.scale(), trueScale, vi.gravityMag(), vi.confidence());
  printf("gravity [%.3f %.3f %.3f]\n", vi.gravity().x(), vi.gravity().y(), vi.gravity().z());

  const double scaleErr = std::abs(vi.scale() - trueScale) / trueScale;
  const double gErr = std::abs(vi.gravityMag() - 9.81);
  const double gDir = (vi.gravity().normalized() - g.normalized()).norm();

  // Tracking loss: window clears, but the last good estimate is retained.
  vi.onLost();
  const bool retained = vi.ok() && vi.windowKeyframes() == 0;
  printf("after onLost: window kf=%d  ok=%d (retained=%d)\n", vi.windowKeyframes(), vi.ok(), retained);

  const bool pass = vi.ok() && scaleErr < 0.05 && gErr < 0.3 && gDir < 0.1 && retained;
  printf("scaleErr %.3f  gErr %.3f  gDir %.3f\n", scaleErr, gErr, gDir);
  printf("%s\n", pass ? "PASS (live VI-init recovers metric scale + gravity)" : "FAIL (VI window)");
  return pass ? 0 : 1;
}
