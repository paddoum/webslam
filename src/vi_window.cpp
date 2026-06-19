#include "vi_window.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace webslam {

using Eigen::Matrix3d;
using Eigen::Vector3d;

void ViWindow::addImu(const ImuSample& s) {
  buf_.push_back(s);
  while (buf_.size() > maxBufferSamples) buf_.pop_front();
}

void ViWindow::snapshot(const Matrix3d& Rwb, const Vector3d& center, bool first) {
  if (!first) {
    // Preintegrate the IMU accumulated since the previous snapshot. Biases are
    // assumed ~0 (M12.0 measured 0.0005 rad/s gyro bias; gravity is solved for).
    std::vector<ImuSample> samples(buf_.begin(), buf_.end());
    pre_.push_back(preintegrate(samples, Vector3d::Zero(), Vector3d::Zero()));
  }
  buf_.clear();
  Rwb_.push_back(Rwb);
  pos_.push_back(center);
  // Slide the window: drop the oldest snapshot AND the segment leaving with it.
  while ((int)Rwb_.size() > maxKeyframes) {
    Rwb_.pop_front();
    pos_.pop_front();
    if (!pre_.empty()) pre_.pop_front();
  }
  lastRwb_ = Rwb;
  lastCenter_ = center;
  haveSnap_ = true;
  framesSinceSnap_ = 0;
}

void ViWindow::onTrackedFrame(const Matrix3d& Rwb, const Vector3d& center) {
  ++framesSinceSnap_;
  if (!haveSnap_) {
    snapshot(Rwb, center, /*first=*/true);
    return;
  }
  const double trans = (center - lastCenter_).norm();
  // Rotation angle of Rwb * lastRwb_^T.
  double tr = (Rwb * lastRwb_.transpose()).trace();
  tr = std::max(-1.0, std::min(3.0, tr));
  const double rotDeg = std::acos(std::max(-1.0, std::min(1.0, (tr - 1.0) / 2.0))) * 180.0 / M_PI;

  const bool moved = trans > snapTrans || rotDeg > snapRotDeg;
  if (framesSinceSnap_ >= minFramesBetween && moved && !buf_.empty()) {
    snapshot(Rwb, center, /*first=*/false);
    if ((int)Rwb_.size() >= minKeyframes) runInit();
  }
}

void ViWindow::runInit() {
  // pre_ holds the between-snapshot segments; invariant: pre_.size()==Rwb_.size()-1.
  if ((int)pre_.size() != (int)Rwb_.size() - 1) return;
  std::vector<Matrix3d> Rwb(Rwb_.begin(), Rwb_.end());
  std::vector<Vector3d> pos(pos_.begin(), pos_.end());
  std::vector<Preintegrated> pre(pre_.begin(), pre_.end());

  ViInit r = viInitialize(Rwb, pos, pre);
  if (!r.ok || !(r.scale > 0) || !std::isfinite(r.scale)) return;
  if (r.gravityMag < gravityLo || r.gravityMag > gravityHi) return;  // implausible -> reject

  result_ = r;
  ok_ = true;
  confidence_ = std::max(0.0, 1.0 - std::abs(r.gravityMag - 9.81) / 3.0);
}

void ViWindow::onLost() {
  // Keep result_/ok_ (last good) but break the preintegration chain.
  buf_.clear();
  Rwb_.clear();
  pos_.clear();
  pre_.clear();
  haveSnap_ = false;
  framesSinceSnap_ = 0;
}

void ViWindow::reset() {
  onLost();
  result_ = ViInit{};
  ok_ = false;
  confidence_ = 0;
}

}  // namespace webslam
