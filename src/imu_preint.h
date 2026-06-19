#pragma once
#include <Eigen/Dense>
#include <vector>

namespace webslam {

// One IMU reading and the time step to the next sample.
struct ImuSample {
  double dt;             // seconds to the next sample
  Eigen::Vector3d gyro;  // measured angular velocity (rad/s, body frame)
  Eigen::Vector3d acc;   // measured specific force (m/s^2, body frame)
};

// Preintegrated IMU between two keyframes, relative to the first body frame.
// Independent of the absolute pose/velocity/gravity — depends only on the biases
// used. This is the Forster-style on-manifold preintegration.
struct Preintegrated {
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();  // ΔR_ij
  Eigen::Vector3d dv = Eigen::Vector3d::Zero();      // Δv_ij
  Eigen::Vector3d dp = Eigen::Vector3d::Zero();      // Δp_ij
  double dt = 0;                                      // total Δt_ij
};

// Integrate the samples with the given gyro/accel biases subtracted.
Preintegrated preintegrate(const std::vector<ImuSample>& samples,
                           const Eigen::Vector3d& bg, const Eigen::Vector3d& ba);

// 9-vector residual relating two VI states (rotation, velocity, position) via a
// preintegrated measurement and gravity g (e.g. [0,0,-9.81]). Zero when the
// states are consistent with the IMU. Layout: [rotation(3); velocity(3); position(3)].
Eigen::Matrix<double, 9, 1> imuResidual(
    const Preintegrated& pre, const Eigen::Vector3d& g,
    const Eigen::Matrix3d& Ri, const Eigen::Vector3d& vi, const Eigen::Vector3d& pi,
    const Eigen::Matrix3d& Rj, const Eigen::Vector3d& vj, const Eigen::Vector3d& pj);

// SO(3) exp/log used by preintegration (exposed for tests).
Eigen::Matrix3d expSO3(const Eigen::Vector3d& w);
Eigen::Vector3d logSO3(const Eigen::Matrix3d& R);

}  // namespace webslam
