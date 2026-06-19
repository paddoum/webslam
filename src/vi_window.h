#pragma once
#include <Eigen/Dense>
#include <deque>

#include "imu_preint.h"
#include "vi_init.h"

namespace webslam {

// Live driver for VINS-Mono-style VI initialization (M12.2) — the open analogue
// of running the bootstrap solve continuously on the device, replacing the
// bolt-on M5/M8 velocity-correlation scale with a one-shot linear solve that
// recovers metric scale, the gravity direction, and per-frame velocity.
//
// It maintains a sliding window of "VI keyframes": snapshots of the visual pose
// (up-to-scale, body=camera) taken when the camera has moved enough, with the
// IMU preintegrated between consecutive snapshots. Once the window is long
// enough it runs viInitialize() and gates the result on a plausible gravity
// magnitude. The window is fed:
//   - addImu(): every high-rate IMU sample (camera frame; biases assumed ~0).
//   - onTrackedFrame(): each frame the visual tracker is healthy.
//   - onLost(): when tracking drops, so preintegration segments stay contiguous.
class ViWindow {
 public:
  // Tunables (visual units; the map's init baseline = 1).
  int minKeyframes = 5;        // viInitialize needs >= 4; keep a margin
  int maxKeyframes = 12;       // sliding-window cap (keeps the linear solve small)
  double snapTrans = 0.08;     // translation since last snapshot to take a new one
  double snapRotDeg = 6.0;     // or rotation since last snapshot
  int minFramesBetween = 2;    // ensure the IMU buffer has accumulated samples
  size_t maxBufferSamples = 2000;  // safety cap on the between-snapshot IMU buffer
  double gravityLo = 7.0, gravityHi = 12.0;  // plausibility gate on |g|

  // Accumulate one IMU sample (camera frame: gyro rad/s, accel m/s^2 incl. gravity).
  void addImu(const ImuSample& s);

  // Offer the current up-to-scale visual pose. Rwb is body(=camera)->world,
  // center is the camera centre in world (visual) units. Decides internally
  // whether to snapshot, and (re)runs the solve when the window is long enough.
  void onTrackedFrame(const Eigen::Matrix3d& Rwb, const Eigen::Vector3d& center);

  // Tracking dropped — reset the window so preintegration stays contiguous. The
  // last good result is retained (so a sphere stays metric across a brief loss).
  void onLost();

  // Forget everything, including the last good result.
  void reset();

  bool ok() const { return ok_; }
  double scale() const { return result_.scale; }           // metres per visual unit
  const Eigen::Vector3d& gravity() const { return result_.gravity; }  // world frame
  double gravityMag() const { return result_.gravityMag; }
  double confidence() const { return confidence_; }         // [0,1], |g|-closeness
  int windowKeyframes() const { return (int)Rwb_.size(); }

 private:
  void snapshot(const Eigen::Matrix3d& Rwb, const Eigen::Vector3d& center, bool first);
  void runInit();

  std::deque<ImuSample> buf_;                 // IMU since the last snapshot
  std::deque<Eigen::Matrix3d> Rwb_;           // snapshot rotations (body->world)
  std::deque<Eigen::Vector3d> pos_;           // snapshot centres (visual units)
  std::deque<Preintegrated> pre_;             // preintegration between snapshots
  Eigen::Matrix3d lastRwb_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d lastCenter_ = Eigen::Vector3d::Zero();
  bool haveSnap_ = false;
  int framesSinceSnap_ = 0;

  ViInit result_;
  bool ok_ = false;
  double confidence_ = 0;
};

}  // namespace webslam
