#include "vi_init.h"

namespace webslam {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

ViInit viInitialize(const std::vector<Matrix3d>& Rwb,
                    const std::vector<Vector3d>& pVisual,
                    const std::vector<Preintegrated>& pre) {
  ViInit out;
  const int frames = (int)Rwb.size();
  const int n = (int)pre.size();  // consecutive pairs
  if (frames < 4 || n != frames - 1 || (int)pVisual.size() != frames) return out;

  // Unknowns: V_0..V_n (3 each), gravity G (3), scale s (1).
  const int gcol = 3 * frames;
  const int scol = gcol + 3;
  const int cols = scol + 1;
  const int rows = 6 * n;
  MatrixXd A = MatrixXd::Zero(rows, cols);
  VectorXd b = VectorXd::Zero(rows);
  const Matrix3d I = Matrix3d::Identity();

  for (int k = 0; k < n; ++k) {
    const int i = k, j = k + 1, rb = 6 * k;
    const double dt = pre[k].dt;
    const Matrix3d& Ri = Rwb[i];

    // Velocity rows:  V_i - V_j + G·dt = -R_i·Δv
    A.block<3, 3>(rb, 3 * i) += I;
    A.block<3, 3>(rb, 3 * j) += -I;
    A.block<3, 3>(rb, gcol) += dt * I;
    b.segment<3>(rb) = -Ri * pre[k].dv;

    // Position rows: s·(p_ci - p_cj) + V_i·dt + 0.5·dt²·G = -R_i·Δp
    A.block<3, 3>(rb + 3, 3 * i) += dt * I;
    A.block<3, 3>(rb + 3, gcol) += 0.5 * dt * dt * I;
    A.block<3, 1>(rb + 3, scol) += (pVisual[i] - pVisual[j]);
    b.segment<3>(rb + 3) = -Ri * pre[k].dp;
  }

  VectorXd x = A.colPivHouseholderQr().solve(b);
  if (!x.allFinite()) return out;

  out.scale = x(scol);
  out.gravity = x.segment<3>(gcol);
  out.gravityMag = out.gravity.norm();
  out.velocities.resize(frames);
  for (int i = 0; i < frames; ++i) out.velocities[i] = x.segment<3>(3 * i);
  out.ok = out.scale > 0 && x.head(3 * frames).allFinite();
  return out;
}

}  // namespace webslam
