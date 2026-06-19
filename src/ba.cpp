#include "ba.h"

#include <cmath>

namespace webslam {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace {

Matrix3d skew(const Vector3d& v) {
  Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

// Mean reprojection error (pixels) over all observations.
double meanCost(const BAProblem& p) {
  double sum = 0;
  int n = 0;
  for (const auto& o : p.obs) {
    const BACamera& c = p.cameras[o.cam];
    Vector3d Xc = c.R * p.points[o.pt] + c.t;
    if (Xc.z() < 1e-6) continue;
    Vector2d proj(p.K.fx * Xc.x() / Xc.z() + p.K.cx, p.K.fy * Xc.y() / Xc.z() + p.K.cy);
    sum += (proj - o.uv).norm();
    ++n;
  }
  return n ? sum / n : 0.0;
}

}  // namespace

BAStats bundleAdjust(BAProblem& problem, int maxIterations, double huberPx) {
  BAStats stats;
  const int nCam = (int)problem.cameras.size();
  const int nPt = (int)problem.points.size();
  if (nCam == 0 || nPt == 0 || problem.obs.empty()) return stats;

  // Map each non-fixed camera to a slot in the reduced system.
  std::vector<int> camSlot(nCam, -1);
  int nFree = 0;
  for (int c = 0; c < nCam; ++c)
    if (!problem.cameras[c].fixed) camSlot[c] = nFree++;
  if (nFree == 0) return stats;  // nothing to optimize

  stats.initialCost = meanCost(problem);
  double lambda = 1e-3;
  const Intrinsics& K = problem.K;

  for (int iter = 0; iter < maxIterations; ++iter) {
    // Per-camera blocks U (6x6) and g_c (6); per-point V (3x3), g_p (3);
    // per-observation W (6x3) linking its camera and point.
    std::vector<Eigen::Matrix<double, 6, 6>> U(nFree, Eigen::Matrix<double, 6, 6>::Zero());
    std::vector<Eigen::Matrix<double, 6, 1>> gc(nFree, Eigen::Matrix<double, 6, 1>::Zero());
    std::vector<Matrix3d> V(nPt, Matrix3d::Zero());
    std::vector<Vector3d> gp(nPt, Vector3d::Zero());
    // W contributions stored per point: list of (slot, 6x3).
    std::vector<std::vector<std::pair<int, Eigen::Matrix<double, 6, 3>>>> Wp(nPt);

    for (const auto& o : problem.obs) {
      const BACamera& cam = problem.cameras[o.cam];
      const Vector3d& X = problem.points[o.pt];
      Vector3d Xc = cam.R * X + cam.t;
      if (Xc.z() < 1e-6) continue;
      const double invz = 1.0 / Xc.z();
      Vector2d proj(K.fx * Xc.x() * invz + K.cx, K.fy * Xc.y() * invz + K.cy);
      Vector2d e = proj - o.uv;

      // d(proj)/d(Xc)  (2x3)
      Eigen::Matrix<double, 2, 3> Jp_xc;
      Jp_xc << K.fx * invz, 0, -K.fx * Xc.x() * invz * invz,
               0, K.fy * invz, -K.fy * Xc.y() * invz * invz;

      // Huber weight.
      double r = e.norm();
      double w = (r <= huberPx) ? 1.0 : huberPx / r;

      // Point Jacobian: dXc/dX = R  ->  J_pt = Jp_xc * R  (2x3)
      Eigen::Matrix<double, 2, 3> Jpt = Jp_xc * cam.R;
      V[o.pt] += w * Jpt.transpose() * Jpt;
      gp[o.pt] += -w * Jpt.transpose() * e;

      const int slot = camSlot[o.cam];
      if (slot >= 0) {
        // Camera Jacobian: dXc/dxi = [I | -skew(Xc)]  ->  J_cam (2x6)
        Eigen::Matrix<double, 2, 6> Jcam;
        Jcam.block<2, 3>(0, 0) = Jp_xc;
        Jcam.block<2, 3>(0, 3) = Jp_xc * (-skew(Xc));
        U[slot] += w * Jcam.transpose() * Jcam;
        gc[slot] += -w * Jcam.transpose() * e;
        Wp[o.pt].push_back({slot, w * Jcam.transpose() * Jpt});
      }
    }

    // Build the reduced camera system S dc = b via Schur complement.
    const int N = 6 * nFree;
    MatrixXd S = MatrixXd::Zero(N, N);
    VectorXd b = VectorXd::Zero(N);
    for (int s = 0; s < nFree; ++s) {
      Eigen::Matrix<double, 6, 6> Us = U[s];
      Us.diagonal().array() += lambda * Us.diagonal().array().max(1e-9);  // LM damping
      S.block<6, 6>(6 * s, 6 * s) += Us;
      b.segment<6>(6 * s) += gc[s];
    }
    // Precompute damped V^{-1} per point and subtract point contributions.
    std::vector<Matrix3d> Vinv(nPt);
    for (int p = 0; p < nPt; ++p) {
      Matrix3d Vd = V[p];
      Vd.diagonal().array() += lambda * Vd.diagonal().array().max(1e-9);
      Vinv[p] = Vd.inverse();
      const auto& ws = Wp[p];
      for (size_t a = 0; a < ws.size(); ++a) {
        Eigen::Matrix<double, 6, 3> WaVinv = ws[a].second * Vinv[p];
        b.segment<6>(6 * ws[a].first) -= WaVinv * gp[p];
        for (size_t c = 0; c < ws.size(); ++c)
          S.block<6, 6>(6 * ws[a].first, 6 * ws[c].first) -= WaVinv * ws[c].second.transpose();
      }
    }

    VectorXd dc = S.ldlt().solve(b);
    if (!dc.allFinite()) { lambda *= 10; continue; }

    // Back-substitute for point updates.
    std::vector<Vector3d> dp(nPt, Vector3d::Zero());
    for (int p = 0; p < nPt; ++p) {
      Vector3d rhs = gp[p];
      for (const auto& wpair : Wp[p]) rhs -= wpair.second.transpose() * dc.segment<6>(6 * wpair.first);
      dp[p] = Vinv[p] * rhs;
    }

    // Build a trial state and test the cost (LM accept/reject).
    BAProblem trial = problem;
    for (int c = 0; c < nCam; ++c) {
      const int s = camSlot[c];
      if (s < 0) continue;
      Vector3d dr = dc.segment<3>(6 * s);
      Vector3d dphi = dc.segment<3>(6 * s + 3);
      double ang = dphi.norm();
      Matrix3d dR = Matrix3d::Identity();
      if (ang > 1e-12) dR = Eigen::AngleAxisd(ang, dphi / ang).toRotationMatrix();
      trial.cameras[c].R = dR * problem.cameras[c].R;
      trial.cameras[c].t = dR * problem.cameras[c].t + dr;
    }
    for (int p = 0; p < nPt; ++p) trial.points[p] = problem.points[p] + dp[p];

    const double before = meanCost(problem);
    const double after = meanCost(trial);
    if (after < before) {
      problem = trial;       // accept
      lambda = std::max(lambda * 0.5, 1e-7);
      stats.iterations = iter + 1;
      if (before - after < 1e-4) break;  // converged
    } else {
      lambda *= 4;           // reject, damp harder
      if (lambda > 1e8) break;
    }
  }

  stats.finalCost = meanCost(problem);
  return stats;
}

}  // namespace webslam
