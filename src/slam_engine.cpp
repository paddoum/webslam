#include "slam_engine.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>

#include "map.h"
#include "scale.h"
#include "two_view.h"
#include "vi_window.h"

namespace webslam {

namespace {

// The 16-pixel Bresenham circle of radius 3 used by FAST, in clockwise order.
constexpr int kCircleDx[16] = {0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1};
constexpr int kCircleDy[16] = {-3, -3, -2, -1, 0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3};

// FAST corner test: is there a contiguous arc of >= n pixels on the circle
// that are all brighter than p + t, or all darker than p - t?
// Returns the corner score (sum of absolute intensity differences) if a
// corner, otherwise 0.
inline float fastScore(const std::uint8_t* gray, int stride, int x, int y, int t) {
  const int p = gray[y * stride + x];
  const int hi = p + t;
  const int lo = p - t;

  int circle[16];
  for (int i = 0; i < 16; ++i) {
    circle[i] = gray[(y + kCircleDy[i]) * stride + (x + kCircleDx[i])];
  }

  // Quick rejection: a 9-pixel arc needs at least 2 of the 4 compass pixels
  // (indices 0,4,8,12) on the same side. (The classic 3-of-4 high-speed test
  // is only valid for FAST-12, not FAST-9 — it wrongly rejects 90° corners.)
  int brighter4 = 0, darker4 = 0;
  for (int i = 0; i < 16; i += 4) {
    if (circle[i] > hi) ++brighter4;
    else if (circle[i] < lo) ++darker4;
  }
  if (brighter4 < 2 && darker4 < 2) return 0.0f;

  // Full segment test: scan 16 + 9 to handle wrap-around, find longest run.
  auto longestRun = [&](bool brighter) -> int {
    int best = 0, run = 0;
    for (int i = 0; i < 16 + 9; ++i) {
      const int v = circle[i & 15];
      const bool ok = brighter ? (v > hi) : (v < lo);
      if (ok) {
        ++run;
        best = std::max(best, run);
      } else {
        run = 0;
      }
    }
    return best;
  };

  const bool isCorner = longestRun(true) >= 9 || longestRun(false) >= 9;
  if (!isCorner) return 0.0f;

  // Score = total absolute difference around the circle (cheap, monotonic).
  float s = 0.0f;
  for (int i = 0; i < 16; ++i) s += std::abs(circle[i] - p);
  return s;
}

// Wall-clock milliseconds since an arbitrary epoch (portable: native + WASM).
inline double nowMs() {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

std::vector<Keypoint> detectFAST(const std::uint8_t* gray, int width, int height,
                                 int threshold, int maxFeatures) {
  const int stride = width;
  const int margin = 3;
  std::vector<float> score(static_cast<size_t>(width) * height, 0.0f);

  // Pass 1: score every pixel that passes the FAST test.
  for (int y = margin; y < height - margin; ++y) {
    for (int x = margin; x < width - margin; ++x) {
      score[y * stride + x] = fastScore(gray, stride, x, y, threshold);
    }
  }

  // Pass 2: 3x3 non-maximum suppression — keep only local score maxima.
  std::vector<Keypoint> kps;
  for (int y = margin; y < height - margin; ++y) {
    for (int x = margin; x < width - margin; ++x) {
      const float s = score[y * stride + x];
      if (s <= 0.0f) continue;
      bool isMax = true;
      for (int dy = -1; dy <= 1 && isMax; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;
          if (score[(y + dy) * stride + (x + dx)] > s) { isMax = false; break; }
        }
      }
      if (isMax) kps.push_back({static_cast<float>(x), static_cast<float>(y), s});
    }
  }

  // Keep the strongest `maxFeatures` corners (global top-K).
  // NOTE (Phase 3): a grid-based per-cell selection was tried here to spread
  // features spatially, but on the orbit bench clip it *regressed* tracking
  // badly (lost 0->48, orbit inliers ->0) — during an orbit it spends budget on
  // cells that no longer overlap the map, starving the inliers the tracker needs.
  // Reverted to global top-K. Spatial-spread selection stays a possible
  // robustness experiment, but must only spread among map-overlapping regions
  // and be validated on its own, not bundled into a perf pass. See docs/bench.
  if (static_cast<int>(kps.size()) > maxFeatures) {
    std::nth_element(kps.begin(), kps.begin() + maxFeatures, kps.end(),
                     [](const Keypoint& a, const Keypoint& b) { return a.score > b.score; });
    kps.resize(maxFeatures);
  }
  return kps;
}

SlamEngine::SlamEngine(int maxWidth, int maxHeight, int maxFeatures)
    : max_width_(maxWidth),
      max_height_(maxHeight),
      max_features_(maxFeatures) {
  input_rgba_.assign(static_cast<size_t>(maxWidth) * maxHeight * 4, 0);
  gray_.assign(static_cast<size_t>(maxWidth) * maxHeight, 0);
  depth_buf_.assign(static_cast<size_t>(maxWidth) * maxHeight, 0);
  xfeat_kp_.assign(static_cast<size_t>(kMaxXFeat) * 2, 0.0f);
  xfeat_desc_.assign(static_cast<size_t>(kMaxXFeat) * 64, 0.0f);
  keypoints_.reserve(maxFeatures);
  keypoints_flat_.reserve(maxFeatures * 3);
  scale_ = std::make_unique<MetricScaleEstimator>();
  vi_ = std::make_unique<ViWindow>();
}

SlamEngine::~SlamEngine() = default;  // out-of-line: SlamMap/MetricScaleEstimator complete here

void SlamEngine::addScaleSample(double t, double cx, double cy, double cz,
                                double qw, double qx, double qy, double qz,
                                double ax, double ay, double az) {
  Eigen::Matrix3d Rwc = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
  scale_->addSample(t, Eigen::Vector3d(cx, cy, cz), Rwc, Eigen::Vector3d(ax, ay, az));
}
double SlamEngine::metricScale() const { return scale_->result().scale; }
double SlamEngine::scaleConfidence() const { return scale_->result().confidence; }
bool SlamEngine::scaleValid() const { return scale_->result().valid; }
int SlamEngine::scaleSamples() const { return scale_->result().samples; }
int SlamEngine::scaleAxis() const { return scale_->result().axis; }
void SlamEngine::resetScale() { scale_->reset(); }

void SlamEngine::addImuSample(double dt, double gx, double gy, double gz,
                              double ax, double ay, double az) {
  ImuSample s;
  s.dt = dt;
  s.gyro = Eigen::Vector3d(gx, gy, gz);
  s.acc = Eigen::Vector3d(ax, ay, az);
  vi_->addImu(s);
}
bool SlamEngine::viOk() const { return vi_->ok(); }
double SlamEngine::viScale() const { return vi_->scale(); }
double SlamEngine::viGravityX() const { return vi_->gravity().x(); }
double SlamEngine::viGravityY() const { return vi_->gravity().y(); }
double SlamEngine::viGravityZ() const { return vi_->gravity().z(); }
double SlamEngine::viGravityMag() const { return vi_->gravityMag(); }
double SlamEngine::viConfidence() const { return vi_->confidence(); }
int SlamEngine::viKeyframes() const { return vi_->windowKeyframes(); }
void SlamEngine::resetVi() { vi_->reset(); }

std::uintptr_t SlamEngine::inputBufferPtr() const {
  return reinterpret_cast<std::uintptr_t>(input_rgba_.data());
}

void SlamEngine::toGrayscale(int width, int height) {
  const std::uint8_t* src = input_rgba_.data();
  std::uint8_t* dst = gray_.data();
  const int n = width * height;
  for (int i = 0; i < n; ++i) {
    const int r = src[i * 4 + 0];
    const int g = src[i * 4 + 1];
    const int b = src[i * 4 + 2];
    // Rec. 601 luma, integer approximation.
    dst[i] = static_cast<std::uint8_t>((77 * r + 150 * g + 29 * b) >> 8);
  }
}

void SlamEngine::setIntrinsics(double fx, double fy, double cx, double cy) {
  fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
  tracking_ = true;
}

void SlamEngine::enableMapping() {
  Intrinsics K{fx_, fy_, cx_, cy_};
  map_ = std::make_unique<SlamMap>(K);
  // Re-apply the budget override — enableMapping() recreates the map (replay
  // calls it to reset), which would otherwise silently revert to the default.
  if (max_map_points_override_ > 0) map_->maxMapPoints = max_map_points_override_;
  if (hold_frames_set_) map_->holdMaxFrames = hold_frames_override_;
}

void SlamEngine::setMaxMapPoints(int n) {
  max_map_points_override_ = std::max(500, n);
  if (map_) map_->maxMapPoints = max_map_points_override_;
}

void SlamEngine::setHoldFrames(int n) {
  hold_frames_override_ = std::max(0, std::min(120, n));
  hold_frames_set_ = true;
  if (map_) map_->holdMaxFrames = hold_frames_override_;
}

void SlamEngine::setAnchor(double x, double y, double z) {
  if (map_) map_->setAnchor(Eigen::Vector3d(x, y, z));
}
bool SlamEngine::anchorValid() const { return map_ && map_->anchorValid(); }
double SlamEngine::anchorX() const { return map_ ? map_->anchorWorld().x() : 0; }
double SlamEngine::anchorY() const { return map_ ? map_->anchorWorld().y() : 0; }
double SlamEngine::anchorZ() const { return map_ ? map_->anchorWorld().z() : 0; }

void SlamEngine::setXFeat(int n) {
  if (!map_) return;
  if (n < 0) n = 0;
  if (n > kMaxXFeat) n = kMaxXFeat;
  map_->setFrameXFeat(xfeat_kp_.data(), xfeat_desc_.data(), n);
}

void SlamEngine::setMotionHint(double px) { if (map_) map_->setSearchHint(px); }

void SlamEngine::setGyroDelta(double wx, double wy, double wz, double dt) {
  if (map_) map_->setGyroDelta(Eigen::Vector3d(wx, wy, wz), dt);
}

int SlamEngine::densifyFromDepth(int dw, int dh, double a, double b) {
  if (!map_) return 0;
  int added = map_->densifyFromDepth(depth_buf_.data(), dw, dh, last_w_, last_h_, a, b, cur_);
  map_num_points_ = map_->numPoints();
  return added;
}

int SlamEngine::processFrame(int width, int height, int threshold) {
  if (width > max_width_ || height > max_height_) return 0;
  last_w_ = width; last_h_ = height;

  // Reset per-frame timings (mapFrame fills [2]=orb, [3]=mapProcess).
  for (float& s : stage_times_) s = 0.0f;
  const double t0 = nowMs();

  toGrayscale(width, height);
  const double t1 = nowMs();
  keypoints_ = detectFAST(gray_.data(), width, height, threshold, max_features_);
  const double t2 = nowMs();

  // Flatten for JS.
  keypoints_flat_.clear();
  for (const auto& kp : keypoints_) {
    keypoints_flat_.push_back(kp.x);
    keypoints_flat_.push_back(kp.y);
    keypoints_flat_.push_back(kp.score);
  }

  if (map_) {
    mapFrame(width, height);
  } else if (tracking_) {
    trackFrame(width, height);
  }

  stage_times_[0] = static_cast<float>(t1 - t0);   // grayscale
  stage_times_[1] = static_cast<float>(t2 - t1);   // detect
  stage_times_[4] = static_cast<float>(nowMs() - t0);  // total
  return static_cast<int>(keypoints_.size());
}

// M3: describe the frame and feed it to the persistent map (init -> PnP track).
void SlamEngine::mapFrame(int width, int height) {
  const double to0 = nowMs();
  computeOrb(gray_.data(), width, height, keypoints_, /*border=*/18, cur_);
  const double to1 = nowMs();
  map_->process(cur_);
  stage_times_[2] = static_cast<float>(to1 - to0);        // orb
  stage_times_[3] = static_cast<float>(nowMs() - to1);    // map process (track + KF/BA)

  map_state_ = static_cast<int>(map_->state());
  map_tracked_ = map_->tracked();
  map_num_points_ = map_->numPoints();
  map_num_kf_ = map_->numKeyframes();
  map_spread_ = map_->mapSpread();
  map_yaw_deg_ = map_->coverageYawDeg();
  map_pitch_deg_ = map_->coveragePitchDeg();
  track_inliers_ = map_->lastInliers();

  // Current pose (world -> camera).
  const Eigen::Matrix3d& R = map_->R();
  const Eigen::Vector3d& t = map_->t();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) pose_r_[i * 3 + j] = (float)R(i, j);
  pose_t_ = {(float)t.x(), (float)t.y(), (float)t.z()};

  // M12.4: drive the live VI-init window. Body=camera, so Rwb = R^T (cam->world);
  // the camera centre is the up-to-scale visual position. Only feed healthy
  // tracking; on loss, break the preintegration chain (keep the last estimate).
  if (map_tracked_ && map_->state() == SlamMap::State::kTracking) {
    vi_->onTrackedFrame(R.transpose(), map_->cameraCenter());
  } else {
    vi_->onLost();
  }

  // Map points (world frame).
  map_points_.clear();
  for (const auto& p : map_->points()) {
    map_points_.push_back((float)p.X.x());
    map_points_.push_back((float)p.X.y());
    map_points_.push_back((float)p.X.z());
  }

  // Estimated trajectory (camera centres).
  trajectory_.clear();
  for (const auto& c : map_->trajectory()) {
    trajectory_.push_back((float)c.x());
    trajectory_.push_back((float)c.y());
    trajectory_.push_back((float)c.z());
  }
}

// M2: describe the current frame, match against the previous frame, and
// recover the relative camera pose. Mirrors FrameFrameTracker.
void SlamEngine::trackFrame(int width, int height) {
  computeOrb(gray_.data(), width, height, keypoints_, /*border=*/18, cur_);

  match_lines_.clear();
  track_ok_ = false;
  track_inliers_ = 0;
  rel_rot_deg_ = 0.0;

  if (prev_.size() >= 8 && cur_.size() >= 8) {
    auto dm = matchHamming(prev_, cur_, /*maxDist=*/64, /*ratio=*/0.8f, /*crossCheck=*/true);
    if (dm.size() >= 8) {
      std::vector<Match2D> mv;
      mv.reserve(dm.size());
      for (const auto& m : dm) {
        mv.push_back({Eigen::Vector2d(prev_.x[m.a], prev_.y[m.a]),
                      Eigen::Vector2d(cur_.x[m.b], cur_.y[m.b])});
      }
      Intrinsics K{fx_, fy_, cx_, cy_};
      RelativePose rp = estimateRelativePose(mv, K);

      if (rp.ok) {
        track_ok_ = true;
        track_inliers_ = rp.inliers;
        double tr = (rp.R.trace() - 1.0) / 2.0;
        tr = std::max(-1.0, std::min(1.0, tr));
        rel_rot_deg_ = std::acos(tr) * 180.0 / M_PI;
        tdir_ = {(float)rp.t.x(), (float)rp.t.y(), (float)rp.t.z()};

        // Accumulate rotation: R_world->cur = R_rel * R_world->prev.
        Eigen::Matrix3d Racc;
        Racc << racc_[0], racc_[1], racc_[2], racc_[3], racc_[4], racc_[5],
                racc_[6], racc_[7], racc_[8];
        Racc = rp.R * Racc;
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j) racc_[i * 3 + j] = (float)Racc(i, j);

        // Inlier match segments for visualization.
        for (size_t i = 0; i < mv.size(); ++i) {
          if (rp.inlierMask[i]) {
            match_lines_.push_back((float)mv[i].a.x());
            match_lines_.push_back((float)mv[i].a.y());
            match_lines_.push_back((float)mv[i].b.x());
            match_lines_.push_back((float)mv[i].b.y());
          }
        }
      } else {
        // Pose failed — still show raw matches so the demo is informative.
        for (const auto& m : dm) {
          match_lines_.push_back(prev_.x[m.a]);
          match_lines_.push_back(prev_.y[m.a]);
          match_lines_.push_back(cur_.x[m.b]);
          match_lines_.push_back(cur_.y[m.b]);
        }
      }
    }
  }

  std::swap(prev_, cur_);
}

}  // namespace webslam
