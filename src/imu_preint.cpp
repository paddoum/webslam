#include "imu_preint.h"

#include <cmath>

namespace webslam {

using Eigen::Matrix3d;
using Eigen::Vector3d;

static Matrix3d skew(const Vector3d& v) {
  Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

Matrix3d expSO3(const Vector3d& w) {
  const double a = w.norm();
  if (a < 1e-9) return Matrix3d::Identity() + skew(w);
  const Vector3d k = w / a;
  const Matrix3d K = skew(k);
  return Matrix3d::Identity() + std::sin(a) * K + (1 - std::cos(a)) * K * K;
}

Vector3d logSO3(const Matrix3d& R) {
  const double c = std::max(-1.0, std::min(1.0, (R.trace() - 1.0) / 2.0));
  const double a = std::acos(c);
  if (a < 1e-7) return Vector3d(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1)) * 0.5;
  return (a / (2.0 * std::sin(a))) * Vector3d(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
}

Preintegrated preintegrate(const std::vector<ImuSample>& samples,
                           const Vector3d& bg, const Vector3d& ba) {
  Preintegrated pre;
  for (const auto& s : samples) {
    const double dt = s.dt;
    if (dt <= 0) continue;
    const Vector3d f = s.acc - ba;     // bias-corrected specific force
    const Vector3d w = s.gyro - bg;    // bias-corrected angular velocity
    // Order matters: use the current dR for position/velocity, then advance it.
    pre.dp += pre.dv * dt + 0.5 * (pre.dR * f) * dt * dt;
    pre.dv += (pre.dR * f) * dt;
    pre.dR = pre.dR * expSO3(w * dt);
    pre.dt += dt;
  }
  return pre;
}

Eigen::Matrix<double, 9, 1> imuResidual(
    const Preintegrated& pre, const Vector3d& g,
    const Matrix3d& Ri, const Vector3d& vi, const Vector3d& pi,
    const Matrix3d& Rj, const Vector3d& vj, const Vector3d& pj) {
  const double dt = pre.dt;
  Eigen::Matrix<double, 9, 1> r;
  r.segment<3>(0) = logSO3(pre.dR.transpose() * Ri.transpose() * Rj);
  r.segment<3>(3) = Ri.transpose() * (vj - vi - g * dt) - pre.dv;
  r.segment<3>(6) = Ri.transpose() * (pj - pi - vi * dt - 0.5 * g * dt * dt) - pre.dp;
  return r;
}

}  // namespace webslam
