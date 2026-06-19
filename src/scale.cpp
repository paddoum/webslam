#include "scale.h"

#include <algorithm>
#include <cmath>

namespace webslam {

using Eigen::Matrix3d;
using Eigen::Vector3d;

MetricScaleEstimator::MetricScaleEstimator() {
  // Generate the 24 proper (det=+1) signed axis-permutation rotations — the
  // candidate device->camera alignments.
  const int perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
  for (auto& pm : perms) {
    for (int sgn = 0; sgn < 8; ++sgn) {
      double s[3] = {(sgn & 1) ? -1.0 : 1.0, (sgn & 2) ? -1.0 : 1.0, (sgn & 4) ? -1.0 : 1.0};
      Matrix3d R = Matrix3d::Zero();
      R(0, pm[0]) = s[0];
      R(1, pm[1]) = s[1];
      R(2, pm[2]) = s[2];
      if (std::abs(R.determinant() - 1.0) < 1e-6) candidates_.push_back(R);
    }
  }
}

void MetricScaleEstimator::reset() {
  window_.clear();
  last_ = ScaleResult{};
}

void MetricScaleEstimator::addSample(double t, const Vector3d& visualPos, const Matrix3d& Rwc,
                                     const Vector3d& aDevice) {
  window_.push_back({t, visualPos, Rwc, aDevice});
  while (window_.size() > windowSamples) window_.pop_front();

  last_ = ScaleResult{};
  last_.samples = (int)window_.size();
  if (window_.size() < 8) return;

  // Try every candidate device->camera alignment; keep the best-correlated one.
  double bestConf = -1, bestScale = 0;
  int bestAxis = -1;
  for (size_t c = 0; c < candidates_.size(); ++c) {
    double scale, conf;
    if (!scoreCandidate(candidates_[c], scale, conf)) continue;
    if (conf > bestConf) { bestConf = conf; bestScale = scale; bestAxis = (int)c; }
  }
  if (bestAxis < 0) return;

  last_.scale = bestScale;
  last_.confidence = bestConf;
  last_.axis = bestAxis;
  last_.valid = (bestScale > 0 && bestConf >= minConfidence);
}

bool MetricScaleEstimator::scoreCandidate(const Matrix3d& Rci, double& scale,
                                          double& confidence) const {
  const int n = (int)window_.size();
  // Rotate each device-frame accel into world via the candidate: a_world =
  // Rwc^T * Rci * a_device. Remove the DC component (residual gravity / bias).
  std::vector<Vector3d> aWorld(n);
  Vector3d aMean = Vector3d::Zero();
  for (int i = 0; i < n; ++i) {
    aWorld[i] = window_[i].Rwc.transpose() * (Rci * window_[i].aDev);
    aMean += aWorld[i];
  }
  aMean /= n;

  // Velocity-level alignment: integrate IMU accel once, differentiate visual
  // position once (both single-order — noise isn't blown up).
  std::vector<Vector3d> vImu(n, Vector3d::Zero());
  for (int i = 1; i < n; ++i) {
    const double dt = window_[i].t - window_[i - 1].t;
    if (dt <= 1e-4 || dt > 1.0) { vImu[i] = vImu[i - 1]; continue; }
    Vector3d a0 = aWorld[i - 1] - aMean, a1 = aWorld[i] - aMean;
    vImu[i] = vImu[i - 1] + 0.5 * (a1 + a0) * dt;
  }
  std::vector<Vector3d> vVis(n, Vector3d::Zero());
  for (int i = 1; i + 1 < n; ++i) {
    const double dt2 = window_[i + 1].t - window_[i - 1].t;
    if (dt2 <= 1e-4) continue;
    vVis[i] = (window_[i + 1].p - window_[i - 1].p) / dt2;
  }

  const int lo = 1, hi = n - 1, m = hi - lo;
  if (m < 6) return false;
  Vector3d meanI = Vector3d::Zero(), meanV = Vector3d::Zero();
  for (int i = lo; i < hi; ++i) { meanI += vImu[i]; meanV += vVis[i]; }
  meanI /= m; meanV /= m;

  double Sxx = 0, Syy = 0, Sxy = 0;
  for (int i = lo; i < hi; ++i) {
    Vector3d x = vVis[i] - meanV, y = vImu[i] - meanI;
    Sxx += x.squaredNorm(); Syy += y.squaredNorm(); Sxy += x.dot(y);
  }
  if (Sxx < minVisualAccel || Syy <= 0 || Sxy <= 0) return false;

  // Total least squares slope + cosine similarity.
  scale = (Syy - Sxx + std::sqrt((Syy - Sxx) * (Syy - Sxx) + 4.0 * Sxy * Sxy)) / (2.0 * Sxy);
  confidence = Sxy / std::sqrt(Sxx * Syy);
  return scale > 0;
}

}  // namespace webslam
