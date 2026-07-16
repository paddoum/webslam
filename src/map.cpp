#include "map.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <unordered_map>

namespace webslam {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

// Wrap a list of descriptors as an OrbFeatures so matchHamming can be reused.
OrbFeatures asFeatures(const std::vector<Descriptor>& descs) {
  OrbFeatures f;
  f.desc = descs;
  f.x.assign(descs.size(), 0.0f);
  f.y.assign(descs.size(), 0.0f);
  f.angle.assign(descs.size(), 0.0f);
  return f;
}

}  // namespace

double SlamMap::mapSpread() const {
  if (keyframes_.size() < 2) return 0.0;
  Vector3d mn = keyframes_[0].cameraCenterWorld(), mx = mn;
  for (const auto& kf : keyframes_) {
    Vector3d c = kf.cameraCenterWorld();
    mn = mn.cwiseMin(c);
    mx = mx.cwiseMax(c);
  }
  return (mx - mn).norm();
}

// Range (max-min) of keyframe viewing-direction yaw / pitch in degrees.
// Viewing direction in world = third row of R; yaw = atan2(dx, dz),
// pitch = atan2(dy, hypot(dx, dz)).
static void coverageSpans(const std::vector<Keyframe>& kfs, double& yawDeg, double& pitchDeg) {
  yawDeg = pitchDeg = 0.0;
  if (kfs.size() < 2) return;
  double yMin = 1e9, yMax = -1e9, pMin = 1e9, pMax = -1e9;
  for (const auto& kf : kfs) {
    Vector3d d(kf.R(2, 0), kf.R(2, 1), kf.R(2, 2));
    d.normalize();
    const double yaw = std::atan2(d.x(), d.z());
    const double pitch = std::atan2(d.y(), std::hypot(d.x(), d.z()));
    yMin = std::min(yMin, yaw); yMax = std::max(yMax, yaw);
    pMin = std::min(pMin, pitch); pMax = std::max(pMax, pitch);
  }
  yawDeg = (yMax - yMin) * 180.0 / M_PI;
  pitchDeg = (pMax - pMin) * 180.0 / M_PI;
}

int SlamMap::densifyFromDepth(const std::uint8_t* depth, int dw, int dh, int procW, int procH,
                              double a, double b, const OrbFeatures& f) {
  if (state_ != State::kTracking || dw <= 0 || dh <= 0) return 0;

  // Which current-frame corners already matched an existing map point.
  std::vector<char> matched(f.size(), 0);
  for (const auto& m : lastMapMatches_)
    if (m.b >= 0 && m.b < (int)f.size()) matched[m.b] = 1;

  int added = 0;
  for (size_t i = 0; i < f.size() && added < 250; ++i) {
    if (matched[i]) continue;  // only fill gaps — don't duplicate tracked points
    const int dx = (int)(f.x[i] / procW * dw), dy = (int)(f.y[i] / procH * dh);
    if (dx < 0 || dy < 0 || dx >= dw || dy >= dh) continue;
    const double inv = a * (depth[dy * dw + dx] / 255.0) + b;  // map inverse depth
    if (inv <= 1e-4) continue;
    const double z = 1.0 / inv;
    if (z <= 0 || z > 1e4) continue;
    // Back-project at the current pose: world = R^T (Xc - t).
    Eigen::Vector3d Xc((f.x[i] - K_.cx) / K_.fx * z, (f.y[i] - K_.cy) / K_.fy * z, z);
    MapPoint mp;
    mp.X = curR_.transpose() * (Xc - curt_);
    mp.desc = f.desc[i]; mp.desc2 = mp.desc;
    mp.observations = 1;          // depth-seeded; not yet keyframe-triangulated
    mp.lastSeen = frameCounter_;
    points_.push_back(mp);
    ++added;
  }
  if (added) cullMapPoints();
  return added;
}

double SlamMap::coverageYawDeg() const {
  double y, p; coverageSpans(keyframes_, y, p); return y;
}
double SlamMap::coveragePitchDeg() const {
  double y, p; coverageSpans(keyframes_, y, p); return p;
}

bool SlamMap::process(const OrbFeatures& f) {
  tracked_ = false;
  ++frameCounter_;

  if (state_ == State::kInit) {
    if (tryInitialize(f)) state_ = State::kTracking;
    return tracked_;
  }

  // While tracking, match by PROJECTION (geometry-guided, robust to viewpoint
  // change). If that fails, fall back to global RELOCALIZATION (appearance-only,
  // no prior). This is what lets it survive orbiting/strafing around the scene.
  // Attempt projection tracking from BOTH tracking and lost states: while lost,
  // the pose has been gyro-coasted (below), so as soon as the view overlaps the
  // map again, projection re-acquires — faster and more reliable than the blind
  // global relocalization, which stays as the last resort.
  bool ok = false;
  if (state_ == State::kTracking || state_ == State::kLost) ok = trackByProjection(f);
  if (!ok) {
    // Budgeted global reloc (see relocCooldownFrames in map.h): immediate on the
    // first lost frame, throttled afterwards so lost frames stay cheap and the
    // camera pipeline keeps breathing — projection re-acquire runs every frame.
    if (state_ != State::kLost || framesSinceReloc_ >= relocCooldownFrames) {
      framesSinceReloc_ = 0;
      ok = relocalizeGlobal(f);
    } else {
      ++framesSinceReloc_;
    }
  }

  if (!ok) {
    state_ = State::kLost;
    last_inliers_ = 0;
    motionValid_ = false;
    // Dead-reckon the rotation on the gyro so the next frame's projection has a
    // prior (translation is held — accel double-integration drifts too fast).
    if (haveGyro_) curR_ = gyroDR_ * curR_;
    haveGyro_ = false;
    return false;
  }
  state_ = State::kTracking;

  // --- Motion-gated keyframe insertion. ---
  ++framesSinceKf_;
  if (shouldInsertKeyframe()) {
    insertKeyframe(f, lastMapMatches_);
    framesSinceKf_ = 0;
  }
  haveGyro_ = false;  // consume the prior; main thread sets a fresh one each frame
  refreshAnchor();    // ride BA / VI / scale corrections applied this frame
  return true;
}

// Attach the AR anchor to the K nearest RELIABLE map points (see map.h for
// why a raw world coordinate is not enough). Only solid (real-parallax) and
// repeatedly-observed points qualify: provisional scaffold points have
// unobservable depth that BA later yanks around — attaching to those makes
// the anchor ride the garbage instead of the scene.
void SlamMap::setAnchor(const Eigen::Vector3d& Xw) {
  constexpr int kNeighbors = 8;
  anchorPts_.clear();
  auto gather = [&](auto&& qualifies) {
    std::vector<std::pair<double, int>> byDist;
    for (int i = 0; i < (int)points_.size(); ++i)
      if (qualifies(points_[i])) byDist.push_back({(points_[i].X - Xw).squaredNorm(), i});
    return byDist;
  };
  auto byDist = gather([](const MapPoint& p) { return p.solid && p.observations >= 3; });
  if ((int)byDist.size() < 3) byDist = gather([](const MapPoint& p) { return p.solid; });
  if ((int)byDist.size() < 3) byDist = gather([](const MapPoint&) { return true; });
  if ((int)byDist.size() >= 3) {
    const int k = std::min(kNeighbors, (int)byDist.size());
    std::partial_sort(byDist.begin(), byDist.begin() + k, byDist.end());
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (int j = 0; j < k; ++j) { anchorPts_.push_back(byDist[j].second); c += points_[byDist[j].second].X; }
    c /= k;
    anchorOffset_ = Xw - c;
    double spread = 0;
    for (int j = 0; j < k; ++j) spread += (points_[anchorPts_[j]].X - c).norm();
    anchorSpread0_ = spread / k;
  }
  anchorWorld_ = Xw;
  anchorValid_ = true;
}

// Re-derive the anchor from its attached points' CURRENT positions: centroid +
// the stored offset, rescaled by the neighbourhood spread so global scale
// corrections (monocular drift, VI rescale) carry the anchor too. Falls back
// to the last derived position when too few attached points survive.
void SlamMap::refreshAnchor() {
  if (!anchorValid_ || (int)anchorPts_.size() < 3) return;
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  for (int i : anchorPts_) c += points_[i].X;
  c /= (double)anchorPts_.size();
  double spread = 0;
  for (int i : anchorPts_) spread += (points_[i].X - c).norm();
  spread /= (double)anchorPts_.size();
  double s = (anchorSpread0_ > 1e-9) ? spread / anchorSpread0_ : 1.0;
  s = std::max(0.5, std::min(2.0, s));  // clamp: spread of 8 points is a noisy scale probe
  anchorWorld_ = c + s * anchorOffset_;
}

void SlamMap::setGyroDelta(const Vector3d& camAngVel, double dt) {
  const double th = camAngVel.norm() * dt;
  if (th < 1e-9 || dt <= 0) { gyroDR_ = Matrix3d::Identity(); }
  else {
    // world->camera advances by exp(-[ω]·dt) (left-multiply), matching the
    // codebase's coasting convention (R_t = exp(-[ω]dt)·R_{t-1}).
    gyroDR_ = Eigen::AngleAxisd(-th, camAngVel.normalized()).toRotationMatrix();
  }
  haveGyro_ = true;
}

// Pose prediction. Rotation comes from the GYRO when a prior is set (instant,
// reliable through fast turns); otherwise the constant-velocity model. The
// translation is always constant-velocity (held when no visual motion history).
void SlamMap::predictPose(Matrix3d& R, Vector3d& t) const {
  if (motionValid_) {
    Matrix3d Rrel = curR_ * prevR_.transpose();
    Vector3d trel = curt_ - Rrel * prevt_;
    t = Rrel * curt_ + trel;
    R = haveGyro_ ? (gyroDR_ * curR_) : (Rrel * curR_);
  } else {
    t = curt_;
    R = haveGyro_ ? (gyroDR_ * curR_) : curR_;
  }
}

// Commit a new pose: shift the motion-model history and record the trajectory.
void SlamMap::acceptPose(const Matrix3d& R, const Vector3d& t, bool fromMotion) {
  prevR_ = curR_; prevt_ = curt_;
  curR_ = R; curt_ = t;
  motionValid_ = fromMotion;
  tracked_ = true;
  trajectory_.push_back(cameraCenter());
  if (trajectory_.size() > 3000) trajectory_.erase(trajectory_.begin());
}

bool SlamMap::trackByProjection(const OrbFeatures& f) {
  const int nf = (int)f.size();
  if (points_.empty() || nf < trackMinMatches) return false;

  // Predicted pose from the motion model.
  Matrix3d predR; Vector3d predt;
  predictPose(predR, predt);

  // Adaptive search radius: widen with expected motion so fast movement doesn't
  // push features outside the window. Estimate expected motion as the larger of
  // (a) the constant-velocity prediction's pixel displacement and (b) the gyro
  // hint — (b) reacts instantly to a sudden fast move, (a) handles translation.
  double predMotion = 0;
  {
    std::vector<double> disp;
    const int step = std::max(1, (int)points_.size() / 100);
    for (int i = 0; i < (int)points_.size(); i += step) {
      Vector3d Xp = predR * points_[i].X + predt;
      Vector3d Xc = curR_ * points_[i].X + curt_;
      if (Xp.z() <= 0.1 || Xc.z() <= 0.1) continue;
      const double du = K_.fx * (Xp.x()/Xp.z() - Xc.x()/Xc.z());
      const double dv = K_.fy * (Xp.y()/Xp.z() - Xc.y()/Xc.z());
      disp.push_back(std::sqrt(du*du + dv*dv));
    }
    if (!disp.empty()) { std::nth_element(disp.begin(), disp.begin()+disp.size()/2, disp.end()); predMotion = disp[disp.size()/2]; }
  }
  const double expectedMotion = std::max(predMotion, searchHint_);
  const double R = std::min(trackMaxSearchPx, trackSearchRadiusPx + 1.3 * expectedMotion);

  // Spatial hash of detected features (cell = search radius).
  std::unordered_map<long long, std::vector<int>> grid;
  auto key = [&](int cx, int cy) { return (long long)cy * 100000LL + cx; };
  for (int j = 0; j < nf; ++j)
    grid[key((int)std::floor(f.x[j] / R), (int)std::floor(f.y[j] / R))].push_back(j);

  // For each map point, project with the predicted pose and match to the best
  // nearby detected corner by descriptor distance.
  std::vector<DMatch> mm;
  std::vector<char> used(nf, 0);
  for (int i = 0; i < (int)points_.size(); ++i) {
    Vector3d Xc = predR * points_[i].X + predt;
    if (Xc.z() <= 0.1) continue;
    const double u = K_.fx * Xc.x() / Xc.z() + K_.cx;
    const double v = K_.fy * Xc.y() / Xc.z() + K_.cy;
    const int cx = (int)std::floor(u / R), cy = (int)std::floor(v / R);
    int best = trackMaxDescDist + 1, bestJ = -1;
    for (int dy = -1; dy <= 1; ++dy)
      for (int dx = -1; dx <= 1; ++dx) {
        auto it = grid.find(key(cx + dx, cy + dy));
        if (it == grid.end()) continue;
        for (int j : it->second) {
          const double du = f.x[j] - u, dv = f.y[j] - v;
          if (du * du + dv * dv > R * R) continue;
          int d = std::min(hammingDistance(points_[i].desc,  f.desc[j]),
                           hammingDistance(points_[i].desc2, f.desc[j]));
          if (d < best) { best = d; bestJ = j; }
        }
      }
    if (bestJ >= 0 && !used[bestJ]) { mm.push_back({i, bestJ, best}); used[bestJ] = 1; }
  }
  if ((int)mm.size() < trackMinMatches) return false;

  std::vector<Vector3d> wp; std::vector<Vector2d> uv;
  wp.reserve(mm.size()); uv.reserve(mm.size());
  for (const auto& m : mm) { wp.push_back(points_[m.a].X); uv.push_back(Vector2d(f.x[m.b], f.y[m.b])); }
  PnPResult r = solvePnP(wp, uv, K_, /*pixelThreshold=*/3.0, /*iters=*/60, /*seed=*/9973,
                         /*hasGuess=*/true, predR, predt);
  if (!r.ok || r.inliers < 6) return false;

  acceptPose(r.R, r.t, /*fromMotion=*/true);
  last_inliers_ = r.inliers;
  lastMapMatches_.clear();
  for (size_t k = 0; k < mm.size(); ++k) {
    if (r.inlierMask[k]) {
      points_[mm[k].a].desc2 = points_[mm[k].a].desc;  // save previous viewpoint
      points_[mm[k].a].desc  = f.desc[mm[k].b];         // update to current viewpoint
      ++points_[mm[k].a].observations;
      points_[mm[k].a].lastSeen = frameCounter_;
      lastMapMatches_.push_back(mm[k]);
    }
  }
  return true;
}

bool SlamMap::relocalizeGlobal(const OrbFeatures& f) {
  std::vector<Descriptor> mapDescs;
  mapDescs.reserve(points_.size());
  for (const auto& p : points_) mapDescs.push_back(p.desc);
  OrbFeatures mapFeat = asFeatures(mapDescs);
  auto mm = matchHamming(mapFeat, f, /*maxDist=*/80, /*ratio=*/0.95f, /*crossCheck=*/false);
  if ((int)mm.size() < relocMinInliers) return false;

  std::vector<Vector3d> wp; std::vector<Vector2d> uv;
  for (const auto& m : mm) { wp.push_back(points_[m.a].X); uv.push_back(Vector2d(f.x[m.b], f.y[m.b])); }
  PnPResult r = solvePnP(wp, uv, K_, /*pixelThreshold=*/3.0, /*iters=*/300, /*seed=*/9973,
                         /*hasGuess=*/false);
  if (!r.ok || r.inliers < relocMinInliers) return false;

  acceptPose(r.R, r.t, /*fromMotion=*/false);  // no velocity after a jump
  last_inliers_ = r.inliers;
  lastMapMatches_.clear();
  for (size_t k = 0; k < mm.size(); ++k) {
    if (r.inlierMask[k]) {
      points_[mm[k].a].desc2 = points_[mm[k].a].desc;
      points_[mm[k].a].desc  = f.desc[mm[k].b];
      ++points_[mm[k].a].observations;
      points_[mm[k].a].lastSeen = frameCounter_;
      lastMapMatches_.push_back(mm[k]);
    }
  }
  return true;
}

// Insert a keyframe only when the camera has genuinely moved (or tracking is
// weak) — never just because N frames elapsed while near-stationary. This is
// what stops keyframe/point bloat when the phone is held roughly still.
bool SlamMap::shouldInsertKeyframe() const {
  if (framesSinceKf_ < kfMinFrames) return false;          // avoid bursts
  if (last_inliers_ < kfMinTrackInliers) return true;      // weak tracking
  if (framesSinceKf_ >= kfMaxFrames) return true;          // periodic refresh while moving
  const Keyframe& kf = keyframes_.back();
  const double trans = (cameraCenter() - kf.cameraCenterWorld()).norm();
  Eigen::Matrix3d dR = curR_ * kf.R.transpose();
  double tr = std::max(-1.0, std::min(1.0, (dR.trace() - 1.0) / 2.0));
  const double rotDeg = std::acos(tr) * 180.0 / M_PI;
  return trans > kfMinTranslation || rotDeg > kfMinRotationDeg;
}

// Bound the keyframe count. When culling, prefer to remove KFs whose viewing
// direction is already well-covered by another KF (within kfDiversityDeg). This
// preserves angular coverage during an orbit — the "12 o'clock" KF stays alive
// even as newer ones accumulate, so the relocalizer has candidates all around the
// arc. Fall back to oldest-non-anchor when every remaining KF is unique.
void SlamMap::cullKeyframes() {
  while ((int)keyframes_.size() > maxKeyframes) {
    auto viewDir = [](const Keyframe& kf) -> Eigen::Vector3d {
      return Eigen::Vector3d(kf.R(2, 0), kf.R(2, 1), kf.R(2, 2));
    };
    int toRemove = -1;
    // Scan from oldest non-anchor toward the most recent (skip [0]=anchor, skip last=current).
    for (int i = 1; i < (int)keyframes_.size() - 1; ++i) {
      Eigen::Vector3d di = viewDir(keyframes_[i]);
      for (int j = 1; j < (int)keyframes_.size(); ++j) {
        if (j == i) continue;
        double cosA = std::max(-1.0, std::min(1.0, di.dot(viewDir(keyframes_[j]))));
        double deg = std::acos(cosA) * 180.0 / M_PI;
        if (deg < kfDiversityDeg) { toRemove = i; break; }
      }
      if (toRemove >= 0) break;
    }
    if (toRemove < 0) toRemove = 1;  // every KF is unique — fall back to oldest-first
    keyframes_.erase(keyframes_.begin() + toRemove);
  }
}

bool SlamMap::tryInitialize(const OrbFeatures& f) {
  if (!haveInitRef_) {
    initRef_ = f;
    haveInitRef_ = true;
    return false;
  }

  auto dm = matchHamming(initRef_, f, /*maxDist=*/64, /*ratio=*/0.8f, /*crossCheck=*/true);
  if ((int)dm.size() < initMinInliers) {
    initRef_ = f;  // refresh reference; wait for more overlap
    return false;
  }

  // Require enough parallax (median pixel displacement) before initializing.
  std::vector<double> disp;
  std::vector<Match2D> mv;
  for (const auto& m : dm) {
    Vector2d a(initRef_.x[m.a], initRef_.y[m.a]);
    Vector2d b(f.x[m.b], f.y[m.b]);
    disp.push_back((a - b).norm());
    mv.push_back({a, b});
  }
  std::nth_element(disp.begin(), disp.begin() + disp.size() / 2, disp.end());
  if (disp[disp.size() / 2] < initMinParallaxPx) return false;

  RelativePose rp = estimateRelativePose(mv, K_);
  if (!rp.ok || rp.inliers < initMinInliers) return false;

  // KF0 = identity (world frame); KF1 = recovered pose (scale fixed: |t| = 1).
  Keyframe kf0, kf1;
  kf0.R = Matrix3d::Identity();
  kf0.t = Vector3d::Zero();
  kf0.feat = initRef_;
  kf0.ptIndex.assign(initRef_.size(), -1);
  kf1.R = rp.R;
  kf1.t = rp.t;  // already unit norm
  kf1.feat = f;
  kf1.ptIndex.assign(f.size(), -1);

  // Triangulate inlier correspondences into world (= KF0 camera) frame.
  std::vector<double> reproj;
  for (size_t i = 0; i < mv.size(); ++i) {
    if (!rp.inlierMask[i]) continue;
    Vector2d a = mv[i].a, b = mv[i].b;
    Vector2d xn1((a.x() - K_.cx) / K_.fx, (a.y() - K_.cy) / K_.fy);
    Vector2d xn2((b.x() - K_.cx) / K_.fx, (b.y() - K_.cy) / K_.fy);
    Vector3d X = triangulate(xn1, xn2, rp.R, rp.t);  // in KF0 frame (= world)
    if (X.z() <= 0 || !X.allFinite()) continue;
    Vector3d Xc1 = rp.R * X + rp.t;
    if (Xc1.z() <= 0) continue;  // also in front of KF1

    // Reprojection error of this point in KF1 (init-quality signal).
    Vector2d proj(K_.fx * Xc1.x() / Xc1.z() + K_.cx, K_.fy * Xc1.y() / Xc1.z() + K_.cy);
    reproj.push_back((proj - b).norm());

    MapPoint mp;
    mp.X = X;
    mp.desc = f.desc[dm[i].b]; mp.desc2 = mp.desc;
    mp.observations = 2;
    mp.lastSeen = frameCounter_;
    const int pid = (int)points_.size();
    points_.push_back(mp);
    kf0.ptIndex[dm[i].a] = pid;
    kf1.ptIndex[dm[i].b] = pid;
  }

  // --- Init-quality guard. Reject degenerate/low-quality initializations so we
  // don't seed the map with a bad geometry that never recovers. ---
  if ((int)points_.size() < initMinInliers / 2) {  // too few survived cheirality
    points_.clear();
    return false;
  }
  std::nth_element(reproj.begin(), reproj.begin() + reproj.size() / 2, reproj.end());
  if (reproj[reproj.size() / 2] > initMaxReprojErrPx) {  // poorly conditioned
    points_.clear();
    return false;
  }

  keyframes_.push_back(kf0);
  keyframes_.push_back(kf1);
  curR_ = kf1.R;
  curt_ = kf1.t;
  tracked_ = true;
  last_inliers_ = (int)points_.size();
  framesSinceKf_ = 0;
  trajectory_.clear();
  trajectory_.push_back(kf0.cameraCenterWorld());
  trajectory_.push_back(kf1.cameraCenterWorld());
  return true;
}

void SlamMap::insertKeyframe(const OrbFeatures& f, const std::vector<DMatch>& mapMatches) {
  Keyframe kf;
  kf.R = curR_;
  kf.t = curt_;
  kf.feat = f;
  kf.ptIndex.assign(f.size(), -1);
  // Link features that matched existing map points.
  for (const auto& m : mapMatches) kf.ptIndex[m.b] = m.a;

  // Triangulate NEW points by matching against the previous keyframe.
  const Keyframe& prev = keyframes_.back();
  auto dm = matchHamming(prev.feat, f, /*maxDist=*/64, /*ratio=*/0.8f, /*crossCheck=*/true);

  // Relative pose prev -> cur.
  Matrix3d Rrel = curR_ * prev.R.transpose();
  Vector3d trel = curt_ - Rrel * prev.t;

  const double cosMinParallax = std::cos(triMinParallaxDeg * M_PI / 180.0);
  for (const auto& m : dm) {
    if (prev.ptIndex[m.a] != -1 || kf.ptIndex[m.b] != -1) continue;  // already mapped
    Vector2d a(prev.feat.x[m.a], prev.feat.y[m.a]);
    Vector2d b(f.x[m.b], f.y[m.b]);
    Vector2d xn1((a.x() - K_.cx) / K_.fx, (a.y() - K_.cy) / K_.fy);
    Vector2d xn2((b.x() - K_.cx) / K_.fx, (b.y() - K_.cy) / K_.fy);
    Vector3d Xprev = triangulate(xn1, xn2, Rrel, trel);  // in prev-camera frame
    if (Xprev.z() <= 0 || !Xprev.allFinite()) continue;

    // --- Triangulation quality gates (pan-clip fix, see
    // docs/bench/pan-map-poisoning.md). ---
    // (a) Cheirality in the CURRENT view too (was only checked in prev).
    Vector3d Xcur = Rrel * Xprev + trel;
    if (Xcur.z() <= 0) continue;
    // (b) Reprojection error in BOTH views — hard reject (bad matches).
    const double e1u = K_.fx * Xprev.x() / Xprev.z() + K_.cx - a.x();
    const double e1v = K_.fy * Xprev.y() / Xprev.z() + K_.cy - a.y();
    const double e2u = K_.fx * Xcur.x() / Xcur.z() + K_.cx - b.x();
    const double e2v = K_.fy * Xcur.y() / Xcur.z() + K_.cy - b.y();
    const double err2 = triMaxReprojErrPx * triMaxReprojErrPx;
    if (e1u * e1u + e1v * e1v > err2 || e2u * e2u + e2v * e2v > err2) continue;
    // (c) Parallax: below the minimum ray angle the depth is unobservable —
    //     keep the point as a 2D tracking scaffold but mark it provisional
    //     (!solid) so the capacity cull can't let it evict real geometry.
    //     A HARD gate here kills pan tracking entirely (verified on the pan
    //     clip: lost 162 -> 669); the scaffold is what keeps PnP alive
    //     through a rotation.
    Vector3d ray1 = Xprev.normalized();                      // from prev center
    Vector3d ray2 = (Xprev + Rrel.transpose() * trel).normalized();  // from cur center (in prev frame)
    const bool solid = ray1.dot(ray2) < cosMinParallax;

    Vector3d Xworld = prev.R.transpose() * (Xprev - prev.t);

    MapPoint mp;
    mp.solid = solid;
    mp.X = Xworld;
    mp.desc = f.desc[m.b]; mp.desc2 = mp.desc;
    mp.observations = 2;
    mp.lastSeen = frameCounter_;
    const int pid = (int)points_.size();
    points_.push_back(mp);
    kf.ptIndex[m.b] = pid;
  }
  keyframes_.push_back(kf);

  if (baEnabled) localBundleAdjust();
  cullKeyframes();
  cullMapPoints();  // sliding window — bounds the map while allowing new areas
}

// Sliding-window map: keep the `maxMapPoints` most-recently-observed points and
// drop the rest, compacting the points vector and remapping keyframe indices.
// This lets the camera explore new areas (new points keep getting added) while
// the total stays bounded — old, no-longer-seen regions fall out of the map.
// Solid (well-triangulated) points get a recency bonus so a flood of
// provisional low-parallax scaffold points from a pan can't evict them.
void SlamMap::cullMapPoints() {
  const int n = (int)points_.size();
  if (n <= maxMapPoints) return;

  // Keep-priority (higher = keep):
  //  1. the current working set (seen within cullRecentProtectFrames) outranks
  //     everything — solid or not, tracking is standing on these points;
  //  2. among the rest, recency with a solid bonus: well-triangulated points
  //     age cullSolidBonusFrames slower than provisional low-parallax scaffold,
  //     so a pan's scaffold flood can't evict real geometry.
  // Within each tier, recency (+ bonus) breaks ties — so even if the protected
  // set alone exceeds the budget, the most recent points win, and exploration
  // with a tiny budget still keeps the frontier (see test_explore).
  auto isAnchorPt = [&](int i) {
    return std::find(anchorPts_.begin(), anchorPts_.end(), i) != anchorPts_.end();
  };
  auto key = [&](int i) -> long long {
    const MapPoint& p = points_[i];
    long long k = (long long)p.lastSeen + (p.solid ? cullSolidBonusFrames : 0);
    if (frameCounter_ - p.lastSeen <= cullRecentProtectFrames) k += 1LL << 32;
    if (isAnchorPt(i)) k += 2LL << 32;  // the AR anchor rides these — never cull
    return k;
  };
  std::vector<int> order(n);
  for (int i = 0; i < n; ++i) order[i] = i;
  std::stable_sort(order.begin(), order.end(),
                   [&](int a, int b) { return key(a) > key(b); });

  std::vector<int> remap(n, -1);
  std::vector<MapPoint> kept;
  kept.reserve(maxMapPoints);
  for (int j = 0; j < maxMapPoints; ++j) {
    const int i = order[j];
    remap[i] = (int)kept.size();
    kept.push_back(points_[i]);
  }
  points_ = std::move(kept);

  // Remap every keyframe's feature->point links (culled points become -1).
  for (auto& kf : keyframes_)
    for (int& pi : kf.ptIndex)
      if (pi >= 0) pi = remap[pi];

  // Remap the anchor's attached points (anchor-exempt above, so normally all
  // survive; a shrunk budget could still drop some — refreshAnchor freezes the
  // anchor if fewer than 3 remain).
  if (anchorValid_) {
    std::vector<int> keptAnchor;
    for (int i : anchorPts_) if (remap[i] >= 0) keptAnchor.push_back(remap[i]);
    anchorPts_ = std::move(keptAnchor);
  }
}

// Refine the most recent keyframes and the points they observe with local
// bundle adjustment. The oldest keyframe in the window is fixed to anchor the
// gauge; results are written back into the map.
void SlamMap::localBundleAdjust() {
  const int nkf = (int)keyframes_.size();
  if (nkf < 3) return;
  const int start = std::max(0, nkf - baWindowKeyframes);

  BAProblem prob;
  prob.K = K_;

  // Cameras = window keyframes; fix the earliest (and global KF0 if present).
  std::vector<int> camOf(nkf, -1);
  for (int k = start; k < nkf; ++k) {
    BACamera c;
    c.R = keyframes_[k].R;
    c.t = keyframes_[k].t;
    c.fixed = (k == start);  // anchor
    camOf[k] = (int)prob.cameras.size();
    prob.cameras.push_back(c);
  }

  // Points = those observed by the window keyframes.
  std::vector<int> ptLocal(points_.size(), -1);
  std::vector<int> ptGlobal;  // local -> global
  auto ensurePoint = [&](int gp) {
    if (ptLocal[gp] == -1) {
      ptLocal[gp] = (int)prob.points.size();
      prob.points.push_back(points_[gp].X);
      ptGlobal.push_back(gp);
    }
    return ptLocal[gp];
  };

  for (int k = start; k < nkf; ++k) {
    const Keyframe& kf = keyframes_[k];
    for (size_t i = 0; i < kf.ptIndex.size(); ++i) {
      const int gp = kf.ptIndex[i];
      if (gp < 0) continue;
      const int lp = ensurePoint(gp);
      prob.obs.push_back({camOf[k], lp, Eigen::Vector2d(kf.feat.x[i], kf.feat.y[i])});
    }
  }
  if (prob.points.size() < 8 || prob.obs.size() < 20) return;

  bundleAdjust(prob, /*maxIterations=*/8, /*huberPx=*/2.0);

  // Write back optimized poses and points.
  for (int k = start; k < nkf; ++k) {
    keyframes_[k].R = prob.cameras[camOf[k]].R;
    keyframes_[k].t = prob.cameras[camOf[k]].t;
  }
  for (size_t lp = 0; lp < ptGlobal.size(); ++lp) points_[ptGlobal[lp]].X = prob.points[lp];

  // Adopt the refined pose of the newest keyframe as the current pose.
  curR_ = keyframes_.back().R;
  curt_ = keyframes_.back().t;
}

}  // namespace webslam
