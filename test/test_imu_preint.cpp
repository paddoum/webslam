// Verifies IMU preintegration: synthesize a known trajectory (rotation +
// world motion), generate the IMU measurements it would produce (with biases),
// preintegrate, and confirm the residual against the true endpoint states is
// ~zero — and grows when the states or biases are wrong.
#include <cmath>
#include <cstdio>
#include <vector>

#include "../src/imu_preint.h"

using namespace webslam;
using Eigen::Matrix3d;
using Eigen::Vector3d;

int main() {
  const Vector3d g(0, 0, -9.81);          // gravity
  const double T = 1.0; const int N = 2000; const double dt = T / N;
  const Vector3d w0(0.4, -0.3, 0.6);      // constant body angular velocity
  const Vector3d bg(0.01, -0.02, 0.015);  // true gyro bias
  const Vector3d ba(0.05, -0.03, 0.04);   // true accel bias

  // True world trajectory p(t) (so a_world = p''), and R(t) = Exp(w0 t).
  const Vector3d A(0.6, 0.4, 0.5), wf(2.0, 3.0, 2.5);
  auto pos = [&](double t) { return Vector3d(A.x()*std::sin(wf.x()*t), A.y()*std::sin(wf.y()*t), A.z()*std::sin(wf.z()*t)); };
  auto vel = [&](double t) { return Vector3d(A.x()*wf.x()*std::cos(wf.x()*t), A.y()*wf.y()*std::cos(wf.y()*t), A.z()*wf.z()*std::cos(wf.z()*t)); };
  auto acc = [&](double t) { return Vector3d(-A.x()*wf.x()*wf.x()*std::sin(wf.x()*t), -A.y()*wf.y()*wf.y()*std::sin(wf.y()*t), -A.z()*wf.z()*wf.z()*std::sin(wf.z()*t)); };
  auto rot = [&](double t) { return expSO3(w0 * t); };

  // Synthesize IMU samples: gyro = w0 + bias; acc = R^T(a_world - g) + bias.
  std::vector<ImuSample> samples;
  for (int k = 0; k < N; ++k) {
    const double t = k * dt;
    ImuSample s;
    s.dt = dt;
    s.gyro = w0 + bg;
    s.acc = rot(t).transpose() * (acc(t) - g) + ba;
    samples.push_back(s);
  }

  // Endpoint ground-truth states.
  const Matrix3d Ri = rot(0), Rj = rot(T);
  const Vector3d vi = vel(0), pi = pos(0), vj = vel(T), pj = pos(T);

  // (1) Correct biases → residual ≈ 0 (only discretization error).
  Preintegrated pre = preintegrate(samples, bg, ba);
  auto r = imuResidual(pre, g, Ri, vi, pi, Rj, vj, pj);
  const double rR = r.segment<3>(0).norm(), rV = r.segment<3>(3).norm(), rP = r.segment<3>(6).norm();
  printf("correct bias:  |rR|=%.4f  |rv|=%.4f  |rp|=%.4f\n", rR, rV, rP);

  // (2) Zero bias (wrong) → residual should be large.
  auto r0 = imuResidual(preintegrate(samples, Vector3d::Zero(), Vector3d::Zero()), g, Ri, vi, pi, Rj, vj, pj);
  printf("zero bias:     |rR|=%.4f  |rv|=%.4f  |rp|=%.4f\n",
         r0.segment<3>(0).norm(), r0.segment<3>(3).norm(), r0.segment<3>(6).norm());

  // (3) Perturbed endpoint position → residual should grow.
  auto rPert = imuResidual(pre, g, Ri, vi, pi, Rj, vj, pj + Vector3d(0.2, 0, 0));
  printf("perturbed pj:  |rp|=%.4f (expect ~0.2)\n", rPert.segment<3>(6).norm());

  const bool pass =
      rR < 0.01 && rV < 0.03 && rP < 0.02 &&                 // correct bias ≈ 0
      r0.segment<3>(0).norm() > 0.01 &&                       // wrong bias detected (rotation)
      r0.segment<3>(3).norm() > 0.04 &&                       // wrong bias detected (velocity — most sensitive)
      rPert.segment<3>(6).norm() > 0.15;                      // perturbation detected
  printf("%s\n", pass ? "PASS (IMU preintegration)" : "FAIL (IMU preintegration)");
  return pass ? 0 : 1;
}
