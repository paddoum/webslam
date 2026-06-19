// Verifies visual-inertial initialization: synthesize a known metric trajectory
// (rotation + world motion + gravity), down-scale its positions to fake "visual"
// (up-to-scale) SfM output, generate the IMU preintegration between keyframes,
// then check viInitialize recovers the true scale + gravity (+ velocities).
#include <cmath>
#include <cstdio>
#include <vector>

#include "../src/imu_preint.h"
#include "../src/vi_init.h"

using namespace webslam;
using Eigen::Matrix3d;
using Eigen::Vector3d;

int main() {
  const Vector3d g(0, 0, -9.81);
  const double trueScale = 0.04;          // metres per visual unit
  const Vector3d w0(0.3, -0.2, 0.4);      // body angular velocity

  // True metric world trajectory P(t) (so a_world = P''), R(t) = Exp(w0 t).
  const Vector3d Amp(0.7, 0.5, 0.6), wf(1.6, 2.2, 1.9);
  auto P   = [&](double t){ return Vector3d(Amp.x()*std::sin(wf.x()*t), Amp.y()*std::sin(wf.y()*t), Amp.z()*std::sin(wf.z()*t)); };
  auto V   = [&](double t){ return Vector3d(Amp.x()*wf.x()*std::cos(wf.x()*t), Amp.y()*wf.y()*std::cos(wf.y()*t), Amp.z()*wf.z()*std::cos(wf.z()*t)); };
  auto Acc = [&](double t){ return Vector3d(-Amp.x()*wf.x()*wf.x()*std::sin(wf.x()*t), -Amp.y()*wf.y()*wf.y()*std::sin(wf.y()*t), -Amp.z()*wf.z()*wf.z()*std::sin(wf.z()*t)); };
  auto R   = [&](double t){ return expSO3(w0 * t); };

  const int frames = 8;
  const double kfDt = 0.2, imuDt = 1.0 / 200;  // 200 Hz IMU
  std::vector<Matrix3d> Rwb;
  std::vector<Vector3d> pVisual;
  std::vector<Preintegrated> pre;

  for (int i = 0; i < frames; ++i) {
    const double t = i * kfDt;
    Rwb.push_back(R(t));
    pVisual.push_back(P(t) / trueScale);   // SfM positions are up-to-scale
    if (i < frames - 1) {
      // IMU samples over [t, t+kfDt] from the true motion (gyro=w0, no bias).
      std::vector<ImuSample> s;
      for (double tt = t; tt < t + kfDt - 1e-9; tt += imuDt) {
        ImuSample u; u.dt = imuDt; u.gyro = w0;
        u.acc = R(tt).transpose() * (Acc(tt) - g);   // specific force in body
        s.push_back(u);
      }
      pre.push_back(preintegrate(s, Vector3d::Zero(), Vector3d::Zero()));
    }
  }

  ViInit r = viInitialize(Rwb, pVisual, pre);
  const double scaleErr = std::abs(r.scale - trueScale) / trueScale;
  const double gDirErr = std::acos(std::max(-1.0, std::min(1.0, r.gravity.normalized().dot(g.normalized())))) * 180 / M_PI;
  const double gMagErr = std::abs(r.gravityMag - 9.81);
  // velocity error at frame 0
  const double v0Err = (r.ok ? (r.velocities[0] - V(0)).norm() : 1e9);

  printf("ok=%d  scale=%.4f (true %.4f, err %.1f%%)\n", r.ok, r.scale, trueScale, scaleErr * 100);
  printf("gravity |G|=%.3f (err %.3f)  dir err %.2f deg\n", r.gravityMag, gMagErr, gDirErr);
  printf("velocity[0] err = %.4f m/s\n", v0Err);

  const bool pass = r.ok && scaleErr < 0.05 && gMagErr < 0.3 && gDirErr < 2.0 && v0Err < 0.1;
  printf("%s\n", pass ? "PASS (VI initialization)" : "FAIL (VI initialization)");
  return pass ? 0 : 1;
}
