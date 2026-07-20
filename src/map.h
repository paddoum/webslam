#pragma once
#include <Eigen/Dense>
#include <vector>

#include "ba.h"
#include "orb.h"
#include "pnp.h"
#include "two_view.h"

namespace webslam {

// A 3D landmark in the world map.
struct MapPoint {
  Eigen::Vector3d X;
  Descriptor desc;   // current appearance (most recent matched viewpoint)
  Descriptor desc2;  // previous appearance (older viewpoint) — used as fallback in matching
  int observations;  // how many keyframes see it
  int lastSeen = 0;  // frame index when last matched (for sliding-window culling)
  // Triangulated with real parallax (reliable depth). Low-parallax points from
  // near-pure rotation are kept as a 2D tracking scaffold but marked
  // !solid — the sliding-window cull ages them faster so they can't evict
  // well-triangulated geometry when the map is at capacity.
  bool solid = true;
  // Optional learned (XFeat) descriptor, tagged at keyframe insert when a
  // nearby XFeat keypoint exists. Enables the appearance-robust XFeat
  // relocalization channel; hasX=false points just don't participate in it.
  XDescriptor xdesc{};
  bool hasX = false;
};

// A keyframe: a pose plus the features observed there and their map-point links.
struct Keyframe {
  Eigen::Matrix3d R;  // world -> camera
  Eigen::Vector3d t;
  OrbFeatures feat;
  std::vector<int> ptIndex;  // feature i -> map point index, or -1
  Eigen::Vector3d cameraCenterWorld() const { return -R.transpose() * t; }
};

// The SLAM map orchestrator: initializes a map from two views, then tracks each
// incoming frame against the map by PnP and inserts keyframes (triangulating
// new points) as needed. Open analogue of MapBuilder + MapTracker + PosePnP.
class SlamMap {
 public:
  enum class State { kInit, kTracking, kLost };

  explicit SlamMap(const Intrinsics& K) : K_(K) {}

  // Feed one frame's ORB features. Returns true if the camera is being tracked.
  bool process(const OrbFeatures& f);

  // M9.3: seed map points from aligned monocular depth. For each current-frame
  // corner that did NOT match an existing map point, back-project it using the
  // depth (sampled at its pixel) and the current pose, adding a descriptor-
  // bearing map point. This densifies low-texture areas where triangulation
  // can't keep up. `f` is the current frame's features (same as last process()).
  // `a,b` map net inverse-depth (0..1) to map inverse-depth. Returns # added.
  int densifyFromDepth(const std::uint8_t* depth, int dw, int dh, int procW, int procH,
                       double a, double b, const OrbFeatures& f);

  State state() const { return state_; }
  bool tracked() const { return tracked_; }
  int lastInliers() const { return last_inliers_; }
  int numPoints() const { return (int)points_.size(); }
  int numKeyframes() const { return (int)keyframes_.size(); }
  // Spatial extent the camera has scanned: bounding-box diagonal of keyframe
  // centres, in visual units (the init baseline = 1). A coverage proxy.
  double mapSpread() const;
  // Angular coverage, split into yaw (pan) and pitch (tilt) spans in degrees —
  // the range of keyframe viewing directions about each axis. Scale-free.
  // ARKit-style scanning wants coverage in BOTH, so they're reported separately.
  double coverageYawDeg() const;
  double coveragePitchDeg() const;

  // Current pose (world -> camera) and camera centre in world.
  const Eigen::Matrix3d& R() const { return curR_; }
  const Eigen::Vector3d& t() const { return curt_; }
  Eigen::Vector3d cameraCenter() const { return -curR_.transpose() * curt_; }

  const std::vector<Eigen::Vector3d>& trajectory() const { return trajectory_; }
  const std::vector<MapPoint>& points() const { return points_; }
  // PnP-inlier matches of the latest tracked frame (map point -> feature).
  // These are points V

  // --- AR anchor: attach a world point to nearby map geometry. ---
  // A raw world coordinate drifts relative to the physical scene because BA /
  // VI optimization keep correcting the map (and monocular scale drifts) —
  // the points ride those corrections, a frozen coordinate doesn't. Attaching
  // stores the K nearest map points + the offset from their centroid; every
  // frame the anchor is re-derived from their CURRENT positions (offset
  // rescaled by the neighbourhood's spread to follow scale corrections), so
  // the anchor moves with the scene. Anchor points are exempt from the
  // capacity cull; if they drop below 3 (e.g. map reset), the last derived
  // position is held frozen.
  void setAnchor(const Eigen::Vector3d& Xw);
  bool anchorValid() const { return anchorValid_; }
  const Eigen::Vector3d& anchorWorld() const { return anchorWorld_; }

  // Tunables (sensible defaults; exposed for experiments/tests).
  int initMinInliers = 40;
  double initMinParallaxPx = 3.0;
  double initMaxReprojErrPx = 2.5;   // reject initializations with high reproj error
  int baWindowKeyframes = 7;         // local-BA sliding window size
  bool baEnabled = true;             // run local bundle adjustment on keyframe insert
  int maxMapPoints = 2200;           // sliding-window size: cull least-recently-seen beyond this
  // Triangulation quality gates (KF insert). During near-pure rotation (a pan)
  // the baseline is ~0 and depth is unobservable. Points below the parallax
  // threshold still enter the map (they're a valid 2D tracking scaffold — a
  // hard gate kills pan tracking entirely) but are marked !solid and aged
  // faster by the cull, so they can't evict well-triangulated geometry.
  // Reprojection failures are rejected outright (bad matches).
  double triMinParallaxDeg = 1.0;    // below this ray angle -> provisional (!solid)
  double triMaxReprojErrPx = 3.0;    // max reproj error in BOTH views (hard reject)
  int cullSolidBonusFrames = 300;    // cull priority: solid points age this much slower
  int cullRecentProtectFrames = 40;  // never cull points seen this recently (the
                                     // current working set — even provisional ones;
                                     // otherwise old solid points evict the very
                                     // scaffold tracking is standing on)
  // Motion-gated keyframe insertion (stops bloat when near-stationary).
  int kfMinFrames = 5;               // minimum frames between keyframes
  int kfMaxFrames = 20;              // force a keyframe at least this often while moving
  int kfMinTrackInliers = 25;        // weak tracking -> insert a keyframe
  int kfInsertMinInliers = 20;       // ...but NEVER insert below this (pose unreliable;
                                     // a bad-pose KF poisons the bank + triangulates garbage)
  double kfMinTranslation = 0.12;    // visual units (init baseline = 1) to warrant a KF
  double kfMinRotationDeg = 4.0;
  int maxKeyframes = 80;             // cap; culled by diversity policy (see cullKeyframes)
  double kfDiversityDeg = 20.0;     // min angular gap (°) a KF must have to be kept
  // Relocalization (recover from loss).
  int relocMinInliers = 25;          // strict acceptance to avoid false relocalization
  // Reloc budget: global reloc (full-map brute-force match + 300-iter RANSAC) costs
  // ~100 ms — running it EVERY lost frame starves the camera pipeline on-device,
  // which widens inter-frame motion and prevents the very re-acquisition it's
  // trying to achieve (death spiral seen on the sphere-orbit clip). Attempt it on
  // the FIRST lost frame (brief losses recover instantly), then only every
  // relocCooldownFrames while lost; skipped frames still run the cheap
  // gyro-coasted projection re-acquire.
  int relocCooldownFrames = 4;
  // Keyframe-pose-seeded relocalization: while lost, try projection tracking
  // FROM stored keyframe poses (a few per frame, ranked by viewing-direction
  // alignment with the gyro-coasted heading, round-robin through the rest).
  // Geometry disambiguates repetitive texture (striped rug, wood planks) where
  // the appearance-only brute-force match structurally cannot reach the inlier
  // floor. Far cheaper than global reloc (~3-8 ms/seed vs ~100 ms).
  int relocSeedsPerFrame = 3;        // KF pose hypotheses tried per lost frame
  double relocSeedSearchPx = 70;     // match window around projections from a seed pose
  // XFeat (learned-feature) relocalization channel. Off unless the JS side
  // stages per-frame XFeat features (setFrameXFeat) — inert otherwise, so the
  // default build behaves exactly as before. Matches the lost frame's XFeat
  // descriptors against tagged map-point xdesc (L2, mutual-NN + ratio), then
  // PnP. Learned features stay distinctive on repetitive texture where ORB
  // Hamming matching drowns.
  double xfeatMaxDist2 = 0.6;        // max squared-L2 for a candidate match (2 = orthogonal)
  float  xfeatRatio = 0.9f;          // Lowe ratio on squared-L2 (best < ratio*second)
  double xfeatTagRadiusPx = 4.0;     // KF-tag: map-point pixel -> nearest XFeat kp within this
  // Track-by-projection (robust to viewpoint change while tracking).
  double trackSearchRadiusPx = 36;   // base search window around a predicted projection
  double trackMaxSearchPx = 150;     // adaptive cap for fast motion
  int trackMaxDescDist = 96;         // looser descriptor gate (location constrains it)
  int trackMinMatches = 10;

  // Per-frame hint (px) of expected feature motion — e.g. from the gyro — used
  // to widen the search window the instant the camera moves fast.
  void setSearchHint(double px) { searchHint_ = px; }

  // Stage the current frame's XFeat features (from the JS worker) for the NEXT
  // process() call: consumed once (to tag a keyframe if one is inserted, and to
  // drive relocalizeByXFeat while lost), then cleared. kpxy = 2*n floats (x,y in
  // processing pixels), desc = 64*n floats (L2-normalized). Inert if never called.
  void setFrameXFeat(const float* kpxy, const float* desc, int n);

  // Per-frame gyro rotation prior: the camera-frame angular velocity (rad/s,
  // already rotated by the calibrated device->camera extrinsic) and the time
  // step. Reorients the pose PREDICTION (not just the search radius), so a fast
  // turn doesn't throw the constant-velocity guess off and lose tracking — and
  // so the rotation keeps dead-reckoning while LOST, giving relocalization a
  // prior. Only call when the gyro->camera alignment is calibrated; otherwise
  // the tracker falls back to the constant-velocity model. Valid for one frame.
  void setGyroDelta(const Eigen::Vector3d& camAngVel, double dt);

 private:
  bool tryInitialize(const OrbFeatures& f);
  bool trackByProjection(const OrbFeatures& f);   // motion-model + local search
  // Core of projection tracking: match map points projected from an explicit
  // pose guess, then guess-seeded PnP. minInliers gates acceptance (tracking
  // uses 6; reloc seeds use relocMinInliers to avoid false relocalization).
  bool trackFromPose(const OrbFeatures& f, const Eigen::Matrix3d& R0,
                     const Eigen::Vector3d& t0, double radius, int minInliers,
                     bool fromMotion);
  bool relocalizeByKeyframes(const OrbFeatures& f); // geometry-seeded (see relocSeedsPerFrame)
  bool relocalizeByXFeat();                        // learned-feature match -> PnP (see setFrameXFeat)
  void tagKeyframeXFeat(const Keyframe& kf);       // copy nearby XFeat descs onto the KF's map points
  bool relocalizeGlobal(const OrbFeatures& f);    // global descriptor match, no prior
  void predictPose(Eigen::Matrix3d& R, Eigen::Vector3d& t) const;
  void acceptPose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, bool fromMotion);
  bool shouldInsertKeyframe() const;
  void insertKeyframe(const OrbFeatures& f, const std::vector<DMatch>& mapMatches);
  void cullKeyframes();
  void cullMapPoints();  // sliding window: drop least-recently-seen points, compact indices
  void localBundleAdjust();

  Intrinsics K_;
  State state_ = State::kInit;
  std::vector<MapPoint> points_;
  std::vector<Keyframe> keyframes_;
  OrbFeatures initRef_;
  bool haveInitRef_ = false;

  Eigen::Matrix3d curR_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d curt_ = Eigen::Vector3d::Zero();
  bool tracked_ = false;
  int last_inliers_ = 0;
  int framesSinceKf_ = 0;
  // Constant-velocity motion model: pose of the previous tracked frame.
  Eigen::Matrix3d prevR_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d prevt_ = Eigen::Vector3d::Zero();
  bool motionValid_ = false;
  double searchHint_ = 0;               // expected feature motion (px) this frame
  Eigen::Matrix3d gyroDR_ = Eigen::Matrix3d::Identity();  // world->cam rotation increment (gyro)
  bool haveGyro_ = false;               // a gyro prior is set for this frame
  std::vector<DMatch> lastMapMatches_;  // map-point -> feature, for KF insertion
  int frameCounter_ = 0;                // monotonic frame index (for lastSeen)
  int framesSinceReloc_ = 0;            // lost-frames since the last global-reloc attempt
  int relocSeedCursor_ = 0;             // round-robin position in the ranked KF seed list
  // Staged XFeat features for the current frame (one-shot, set by setFrameXFeat).
  std::vector<Eigen::Vector2f> xfKp_;   // keypoint pixels
  std::vector<XDescriptor> xfDesc_;     // parallel 64-D descriptors
  bool xfValid_ = false;                // staged features available for this process()

  // AR anchor state (see setAnchor).
  void refreshAnchor();                 // re-derive anchor from its points' current positions
  bool anchorValid_ = false;
  Eigen::Vector3d anchorWorld_ = Eigen::Vector3d::Zero();  // last derived position
  std::vector<int> anchorPts_;          // indices into points_ (remapped by cullMapPoints)
  Eigen::Vector3d anchorOffset_ = Eigen::Vector3d::Zero(); // anchor - centroid at attach
  double anchorSpread0_ = 0;            // neighbourhood spread at attach (scale reference)

  std::vector<Eigen::Vector3d> trajectory_;
};

}  // namespace webslam
