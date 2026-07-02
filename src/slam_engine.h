#pragma once
#include <cstdint>
#include <memory>
#include <vector>

#include "fast_features.h"
#include "orb.h"

namespace webslam {

class SlamMap;               // forward-declared; defined in map.h
class MetricScaleEstimator;  // forward-declared; defined in scale.h (keeps Eigen out of this header)
class ViWindow;              // forward-declared; defined in vi_window.h (M12.4: live VI-init)

// The engine: receives RGBA camera frames, runs the vision front-end (FAST +
// ORB), and — once intrinsics are set — frame-to-frame visual tracking that
// recovers the relative camera pose (the open analogue of FrameFrameTracker).
class SlamEngine {
 public:
  SlamEngine(int maxWidth, int maxHeight, int maxFeatures);
  ~SlamEngine();  // out-of-line for unique_ptr<SlamMap>

  // Pointer (as an integer, for JS) to the RGBA input buffer that the page
  // writes each camera frame into. Size = maxWidth * maxHeight * 4 bytes.
  std::uintptr_t inputBufferPtr() const;

  // Direct access to the RGBA staging buffer (for an Embind memory view).
  std::uint8_t* inputData() { return input_rgba_.data(); }
  std::size_t inputSize() const { return input_rgba_.size(); }

  // Provide pinhole intrinsics. Calling this enables M2 frame-to-frame
  // tracking on subsequent processFrame() calls.
  void setIntrinsics(double fx, double fy, double cx, double cy);

  // Enable M3 map-based tracking (init -> PnP against a persistent map).
  // Requires intrinsics; takes precedence over M2 frame-to-frame tracking.
  void enableMapping();

  // M9.3: depth staging buffer the page writes the aligned depth map into.
  std::uint8_t* depthData() { return depth_buf_.data(); }
  std::size_t depthSize() const { return depth_buf_.size(); }
  // Densify the map from the staged depth (dw x dh), with the alignment (a,b).
  int densifyFromDepth(int dw, int dh, double a, double b);

  // Hint (px) of expected feature motion this frame (e.g. gyro-derived) so the
  // tracker widens its search window for fast motion. Call before processFrame.
  void setMotionHint(double px);

  // Per-frame gyro rotation prior (camera-frame angular velocity rad/s, already
  // rotated by the calibrated device->camera extrinsic, + dt). Reorients the
  // pose prediction so fast turns don't lose tracking and recovery has a prior.
  // Call before processFrame, only when the gyro->camera alignment is calibrated.
  void setGyroDelta(double wx, double wy, double wz, double dt);

  // Detect corners (+ track, if intrinsics set) on the staged frame.
  // Returns the number of keypoints found.
  int processFrame(int width, int height, int threshold);

  // Per-frame stage timings in milliseconds (wall clock). Always populated —
  // costs ~5 steady_clock reads/frame, negligible. Order:
  // [grayscale, detect, orb, mapProcess, total]. Use with the .wsrec replay
  // bench mode to get deterministic per-stage p50/p95 without a phone.
  const std::vector<float>& stageTimesFlat() const { return stage_times_; }

  // --- M1 output ---
  const std::vector<float>& keypointsFlat() const { return keypoints_flat_; }

  // --- M2 output ---
  bool trackingOk() const { return track_ok_; }
  int trackingInliers() const { return track_inliers_; }
  double relativeRotationDeg() const { return rel_rot_deg_; }
  // Inlier match segments as [ax, ay, bx, by, ...] (prev -> current pixel).
  const std::vector<float>& matchLinesFlat() const { return match_lines_; }
  // Unit translation direction of the latest relative pose (3 floats).
  const std::vector<float>& translationDir() const { return tdir_; }
  // Accumulated camera rotation since start, row-major 3x3 (9 floats).
  const std::vector<float>& accumulatedRotation() const { return racc_; }

  // --- M3 output (valid when mapping enabled) ---
  int mapState() const { return map_state_; }       // 0=init, 1=tracking, 2=lost
  bool mapTracked() const { return map_tracked_; }
  int mapNumPoints() const { return map_num_points_; }
  int mapNumKeyframes() const { return map_num_kf_; }
  double mapSpread() const { return map_spread_; }  // scanned extent (coverage proxy)
  double coverageYawDeg() const { return map_yaw_deg_; }    // pan coverage
  double coveragePitchDeg() const { return map_pitch_deg_; }  // tilt coverage
  // World-frame map points [x,y,z,...].
  const std::vector<float>& mapPointsFlat() const { return map_points_; }
  // Estimated camera trajectory (centres) [x,y,z,...].
  const std::vector<float>& trajectoryFlat() const { return trajectory_; }
  // Current pose: world->camera rotation (9, row-major) and translation (3).
  const std::vector<float>& poseR() const { return pose_r_; }
  const std::vector<float>& poseT() const { return pose_t_; }

  // --- M5: metric scale from IMU ---
  // Feed one synchronized sample: camera centre (world), the world->camera
  // rotation as a quaternion (qw,qx,qy,qz), and the RAW device-frame linear
  // acceleration (m/s^2, gravity removed). t in seconds. The estimator
  // auto-calibrates the device->camera alignment.
  void addScaleSample(double t, double cx, double cy, double cz,
                      double qw, double qx, double qy, double qz,
                      double ax, double ay, double az);
  double metricScale() const;       // meters per visual unit (raw estimate)
  double scaleConfidence() const;   // [0,1]
  bool scaleValid() const;
  int scaleSamples() const;         // window samples used (diagnostic)
  int scaleAxis() const;            // chosen device->camera alignment index
  void resetScale();

  // --- M12.4: live VI initialization (metric scale + gravity, VINS-Mono style) ---
  // Feed one high-rate IMU sample already rotated into the CAMERA frame (the
  // page applies the calibrated device->camera rotation): gyro (rad/s) and
  // accelerometer specific force INCLUDING gravity (m/s^2). dt = seconds to the
  // next sample. The window preintegrates these between visual keyframes and
  // solves the VI bootstrap once enough motion has accumulated.
  void addImuSample(double dt, double gx, double gy, double gz,
                    double ax, double ay, double az);
  bool viOk() const;                // a plausible solution is available
  double viScale() const;           // metres per visual unit
  double viGravityX() const;        // gravity vector in world (visual) frame
  double viGravityY() const;
  double viGravityZ() const;
  double viGravityMag() const;      // ~9.81 when healthy
  double viConfidence() const;      // [0,1] (|g| closeness)
  int viKeyframes() const;          // snapshots in the current window
  void resetVi();

 private:
  void toGrayscale(int width, int height);
  void trackFrame(int width, int height);
  void mapFrame(int width, int height);

  int max_width_;
  int max_height_;
  int max_features_;

  std::vector<std::uint8_t> input_rgba_;  // staged frame (RGBA)
  std::vector<std::uint8_t> gray_;        // grayscale working buffer
  std::vector<std::uint8_t> depth_buf_;   // staged aligned depth map (M9.3)
  int last_w_ = 0, last_h_ = 0;           // last processed frame dims
  std::vector<Keypoint> keypoints_;
  std::vector<float> keypoints_flat_;     // exposed to JS

  // Tracking state.
  bool tracking_ = false;
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
  OrbFeatures prev_, cur_;
  bool track_ok_ = false;
  int track_inliers_ = 0;
  double rel_rot_deg_ = 0.0;
  std::vector<float> match_lines_;
  std::vector<float> tdir_{0, 0, 0};
  std::vector<float> racc_{1, 0, 0, 0, 1, 0, 0, 0, 1};  // identity

  // M3 mapping.
  std::unique_ptr<SlamMap> map_;
  // M5 metric scale.
  std::unique_ptr<MetricScaleEstimator> scale_;
  // M12.4 live VI-init (metric scale + gravity).
  std::unique_ptr<ViWindow> vi_;
  int map_state_ = 0;
  bool map_tracked_ = false;
  int map_num_points_ = 0;
  int map_num_kf_ = 0;
  double map_spread_ = 0;
  double map_yaw_deg_ = 0;
  double map_pitch_deg_ = 0;
  std::vector<float> map_points_;
  std::vector<float> trajectory_;
  std::vector<float> pose_r_{1, 0, 0, 0, 1, 0, 0, 0, 1};
  std::vector<float> pose_t_{0, 0, 0};

  // [grayscale, detect, orb, mapProcess, total] ms — see stageTimesFlat().
  std::vector<float> stage_times_{0, 0, 0, 0, 0};
};

}  // namespace webslam
