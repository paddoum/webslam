// Verifies bundle adjustment: build a synthetic multi-view problem, perturb the
// (non-fixed) cameras and 3D points, then check BA drives the mean reprojection
// error back down to ~the noise floor and recovers the geometry.
#include <cmath>
#include <cstdio>

#include "../src/ba.h"

using namespace webslam;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

static Matrix3d yaw(double a) {
  Matrix3d R;
  R << std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a);
  return R;
}

int main() {
  Intrinsics K{450, 450, 320, 240};
  unsigned s = 13;
  auto rnd = [&]() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return (s % 100000) / 100000.0; };

  // Ground-truth cameras: 5 views along an arc looking at the scene.
  std::vector<BACamera> gt;
  for (int i = 0; i < 5; ++i) {
    BACamera c;
    double a = (i - 2) * 0.10;             // yaw
    c.R = yaw(a);
    Vector3d C(0.6 * (i - 2), 0.05 * (i - 2), 0.0);  // camera centre
    c.t = -c.R * C;
    gt.push_back(c);
  }

  // Ground-truth points.
  std::vector<Vector3d> gtPts;
  for (int i = 0; i < 120; ++i)
    gtPts.push_back(Vector3d((rnd() - 0.5) * 5, (rnd() - 0.5) * 4, 4 + rnd() * 6));

  // Build observations (with tiny pixel noise) for points visible in each view.
  BAProblem prob;
  prob.K = K;
  prob.cameras = gt;
  prob.points = gtPts;
  for (int ci = 0; ci < (int)gt.size(); ++ci) {
    for (int pi = 0; pi < (int)gtPts.size(); ++pi) {
      Vector3d Xc = gt[ci].R * gtPts[pi] + gt[ci].t;
      if (Xc.z() < 0.5) continue;
      Vector2d uv(K.fx * Xc.x() / Xc.z() + K.cx + (rnd() - 0.5) * 0.4,
                  K.fy * Xc.y() / Xc.z() + K.cy + (rnd() - 0.5) * 0.4);
      if (uv.x() < 0 || uv.y() < 0 || uv.x() >= 640 || uv.y() >= 480) continue;
      prob.obs.push_back({ci, pi, uv});
    }
  }

  // Fix camera 0 (anchor). Perturb the rest + all points.
  prob.cameras[0].fixed = true;
  for (int ci = 1; ci < (int)prob.cameras.size(); ++ci) {
    prob.cameras[ci].t += Vector3d((rnd() - 0.5) * 0.2, (rnd() - 0.5) * 0.2, (rnd() - 0.5) * 0.2);
    prob.cameras[ci].R = yaw((rnd() - 0.5) * 0.15) * prob.cameras[ci].R;
  }
  for (auto& P : prob.points) P += Vector3d((rnd() - 0.5) * 0.4, (rnd() - 0.5) * 0.4, (rnd() - 0.5) * 0.4);

  printf("observations: %zu\n", prob.obs.size());
  BAStats st = bundleAdjust(prob, 25, 2.0);
  printf("mean reprojection error: %.3f px -> %.3f px  (%d iters)\n",
         st.initialCost, st.finalCost, st.iterations);

  // Camera pose recovery vs ground truth (cam0 fixed pins the gauge).
  double maxRot = 0, maxT = 0;
  for (int ci = 1; ci < (int)prob.cameras.size(); ++ci) {
    Matrix3d dR = gt[ci].R.transpose() * prob.cameras[ci].R;
    double tr = std::max(-1.0, std::min(1.0, (dR.trace() - 1) / 2));
    maxRot = std::max(maxRot, std::acos(tr) * 180 / M_PI);
    maxT = std::max(maxT, (prob.cameras[ci].t - gt[ci].t).norm());
  }
  printf("max camera rotation err: %.3f deg   max translation err: %.4f\n", maxRot, maxT);

  const bool pass = st.finalCost < 0.5 && st.finalCost < st.initialCost * 0.2 &&
                    maxRot < 0.5 && maxT < 0.05;
  printf("%s\n", pass ? "PASS (bundle adjustment)" : "FAIL (bundle adjustment)");
  return pass ? 0 : 1;
}
