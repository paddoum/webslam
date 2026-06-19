#include "vi_optimizer.h"

#include <cmath>

namespace webslam {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace {

// State per keyframe: [dp(3), dθ(3), dv(3)]. Biases are NOT estimated here —
// the on-device gyro bias is negligible (M12.0: 0.0005 rad/s) and gravity is
// calibrated, and bias observability needs bias-corrected re-preintegration
// (a documented future refinement). So we fuse pose + velocity.
constexpr int kPerKf = 9;

// Residual weights (sqrt-information). Vision is trusted most; IMU moderate.
constexpr double kWvisR = 8, kWvisP = 8, kWimu = 3;

// Build the stacked, weighted residual vector for the current states.
VectorXd residual(const std::vector<ViState>& s, const std::vector<Preintegrated>& pre,
                  const std::vector<VisualPose>& vis, const Vector3d& g) {
  const int nkf = (int)s.size();
  std::vector<double> r;
  // visual pose residuals
  for (int i = 0; i < nkf; ++i) {
    if (!vis[i].valid) continue;
    Vector3d rR = logSO3(vis[i].R.transpose() * s[i].R) * kWvisR;
    Vector3d rP = (s[i].p - vis[i].p) * kWvisP;
    for (int k = 0; k < 3; ++k) r.push_back(rR[k]);
    for (int k = 0; k < 3; ++k) r.push_back(rP[k]);
  }
  // IMU preintegration residual between consecutive keyframes
  for (int k = 0; k + 1 < nkf; ++k) {
    auto ri = imuResidual(pre[k], g, s[k].R, s[k].v, s[k].p, s[k+1].R, s[k+1].v, s[k+1].p);
    for (int d = 0; d < 9; ++d) r.push_back(ri[d] * kWimu);
  }
  return Eigen::Map<VectorXd>(r.data(), r.size());
}

// Apply a manifold increment to one keyframe (block d0..d0+15 of the full δ).
void applyBlock(ViState& st, const double* d) {
  st.p += Vector3d(d[0], d[1], d[2]);
  st.R = st.R * expSO3(Vector3d(d[3], d[4], d[5]));
  st.v += Vector3d(d[6], d[7], d[8]);
}

}  // namespace

ViOptStats viOptimize(std::vector<ViState>& states, const std::vector<Preintegrated>& pre,
                      const std::vector<VisualPose>& vis, const Vector3d& gravity, int iterations) {
  ViOptStats stats;
  const int nkf = (int)states.size();
  if (nkf < 2 || (int)pre.size() != nkf - 1 || (int)vis.size() != nkf) return stats;

  const int D = kPerKf * nkf;
  // Fix the first keyframe's pose (gauge): its 6 pose dims are not optimized.
  std::vector<int> freeDims;
  for (int i = 0; i < D; ++i) { const int kf = i / kPerKf, loc = i % kPerKf; if (kf == 0 && loc < 6) continue; freeDims.push_back(i); }
  const int F = (int)freeDims.size();

  auto cost = [&](const std::vector<ViState>& s) { return residual(s, pre, vis, gravity).squaredNorm(); };
  stats.initCost = std::sqrt(cost(states) / std::max(1, (int)residual(states, pre, vis, gravity).size()));

  double lambda = 1e-3;
  const double eps = 1e-6;
  VectorXd r0 = residual(states, pre, vis, gravity);
  int R = (int)r0.size();

  for (int it = 0; it < iterations; ++it) {
    r0 = residual(states, pre, vis, gravity);
    // Numerical Jacobian over the free dims (manifold perturbation).
    MatrixXd J(R, F);
    for (int c = 0; c < F; ++c) {
      const int d = freeDims[c], kf = d / kPerKf, loc = d % kPerKf;
      std::vector<ViState> sp = states;
      double inc[kPerKf] = {0}; inc[loc] = eps;
      applyBlock(sp[kf], inc);
      J.col(c) = (residual(sp, pre, vis, gravity) - r0) / eps;
    }
    MatrixXd H = J.transpose() * J;
    VectorXd b = -J.transpose() * r0;
    H.diagonal().array() += lambda * H.diagonal().array().max(1e-9);
    VectorXd dx = H.ldlt().solve(b);
    if (!dx.allFinite()) { lambda *= 10; continue; }

    // trial update
    std::vector<ViState> trial = states;
    VectorXd full = VectorXd::Zero(D);
    for (int c = 0; c < F; ++c) full[freeDims[c]] = dx[c];
    for (int kf = 0; kf < nkf; ++kf) applyBlock(trial[kf], full.data() + kf * kPerKf);

    if (cost(trial) < cost(states)) {
      const double before = cost(states), after = cost(trial);
      states = trial;
      lambda = std::max(lambda * 0.5, 1e-7);
      stats.iterations = it + 1;
      if (before - after < 1e-9) break;
    } else {
      lambda *= 4;
      if (lambda > 1e9) break;
    }
  }
  stats.finalCost = std::sqrt(cost(states) / std::max(1, R));
  return stats;
}

}  // namespace webslam
