#include "pnp.h"

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

namespace webslam {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

struct Rng {
  uint64_t s;
  explicit Rng(unsigned seed) : s(seed ? seed : 0x9e3779b9u) {}
  uint32_t next() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return (uint32_t)s; }
  int below(int n) { return (int)(next() % (uint32_t)n); }
};

Matrix3d skew(const Vector3d& v) {
  Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

// Project a world point into pixels given pose (R,t) and intrinsics.
Vector2d project(const Matrix3d& R, const Vector3d& t, const Intrinsics& K, const Vector3d& X) {
  Vector3d Xc = R * X + t;
  return Vector2d(K.fx * Xc.x() / Xc.z() + K.cx, K.fy * Xc.y() / Xc.z() + K.cy);
}

// Direct Linear Transform PnP from >=6 correspondences (normalized image
// coords). Returns a rough (R,t) with the rotation orthonormalized.
bool dltPnP(const std::vector<Vector3d>& X, const std::vector<Vector2d>& xn,
            Matrix3d& R, Vector3d& t) {
  const int n = (int)X.size();
  if (n < 6) return false;
  MatrixXd A(2 * n, 12);
  A.setZero();
  for (int i = 0; i < n; ++i) {
    const double Xx = X[i].x(), Xy = X[i].y(), Xz = X[i].z();
    Eigen::Vector4d Xh(Xx, Xy, Xz, 1.0);
    const double u = xn[i].x(), v = xn[i].y();
    // u = P1.X / P3.X  -> P1.X - u P3.X = 0
    A.block<1, 4>(2 * i, 0) = Xh.transpose();
    A.block<1, 4>(2 * i, 8) = -u * Xh.transpose();
    A.block<1, 4>(2 * i + 1, 4) = Xh.transpose();
    A.block<1, 4>(2 * i + 1, 8) = -v * Xh.transpose();
  }
  Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeFullV);
  Eigen::VectorXd p = svd.matrixV().col(11);
  Matrix3d R0;
  R0 << p(0), p(1), p(2), p(4), p(5), p(6), p(8), p(9), p(10);
  Vector3d t0(p(3), p(7), p(11));

  // Orthonormalize R0 -> closest rotation; recover scale from its singular vals.
  Eigen::JacobiSVD<Matrix3d> rsvd(R0, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const double scale = rsvd.singularValues().mean();
  if (scale < 1e-9) return false;
  R = rsvd.matrixU() * rsvd.matrixV().transpose();
  if (R.determinant() < 0) R = -R;
  t = t0 / scale;

  // Resolve the global sign so points lie in front of the camera.
  int front = 0;
  for (int i = 0; i < n; ++i)
    if ((R * X[i] + t).z() > 0) ++front;
  if (front < n - front) { t = -t; /* R unchanged: flip handled via depth */ }
  return true;
}

// Gauss-Newton refinement of (R,t) minimizing pixel reprojection error, with
// Huber robustification. Left-multiplicative se(3) update.
void refineGN(const std::vector<Vector3d>& X, const std::vector<Vector2d>& uv,
              const Intrinsics& K, Matrix3d& R, Vector3d& t, int iters = 10) {
  const double huber = 4.0;  // pixels
  for (int it = 0; it < iters; ++it) {
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
    for (size_t i = 0; i < X.size(); ++i) {
      Vector3d Xc = R * X[i] + t;
      if (Xc.z() < 1e-6) continue;
      const double invz = 1.0 / Xc.z();
      Vector2d proj(K.fx * Xc.x() * invz + K.cx, K.fy * Xc.y() * invz + K.cy);
      Vector2d e = proj - uv[i];

      // d(proj)/d(Xc)
      Eigen::Matrix<double, 2, 3> Jc;
      Jc << K.fx * invz, 0, -K.fx * Xc.x() * invz * invz,
            0, K.fy * invz, -K.fy * Xc.y() * invz * invz;
      // d(Xc)/d(xi) = [I | -skew(Xc)]  (left perturbation)
      Eigen::Matrix<double, 3, 6> Jx;
      Jx.block<3, 3>(0, 0) = Matrix3d::Identity();
      Jx.block<3, 3>(0, 3) = -skew(Xc);
      Eigen::Matrix<double, 2, 6> J = Jc * Jx;

      // Huber weight.
      double r = e.norm();
      double w = (r <= huber) ? 1.0 : huber / r;

      H += w * J.transpose() * J;
      b += w * J.transpose() * e;
    }
    Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(-b);
    if (!dx.allFinite()) break;
    // Apply left update: T <- exp(dx) T  =>  t += dr + ... ; R <- exp(dphi) R
    Vector3d dr = dx.head<3>();
    Vector3d dphi = dx.tail<3>();
    double ang = dphi.norm();
    Matrix3d dR = Matrix3d::Identity();
    if (ang > 1e-12) {
      Vector3d axis = dphi / ang;
      dR = Eigen::AngleAxisd(ang, axis).toRotationMatrix();
    }
    R = dR * R;
    t = dR * t + dr;
    if (dx.norm() < 1e-9) break;
  }
}

}  // namespace

PnPResult solvePnP(const std::vector<Vector3d>& worldPts,
                   const std::vector<Vector2d>& imgPts, const Intrinsics& K,
                   double pixelThreshold, int iterations, unsigned seed,
                   bool hasGuess, const Matrix3d& Rinit, const Vector3d& tinit) {
  PnPResult res;
  const int n = (int)worldPts.size();
  if (n < 6) return res;

  // Normalized image coords for the DLT.
  std::vector<Vector2d> xn(n);
  for (int i = 0; i < n; ++i)
    xn[i] = Vector2d((imgPts[i].x() - K.cx) / K.fx, (imgPts[i].y() - K.cy) / K.fy);

  Rng rng(seed);
  Matrix3d bestR = Rinit;
  Vector3d bestT = tinit;
  int bestInliers = -1;
  std::vector<char> bestMask(n, 0);

  auto countInliers = [&](const Matrix3d& R, const Vector3d& t, std::vector<char>& mask) {
    int c = 0;
    for (int i = 0; i < n; ++i) {
      Vector3d Xc = R * worldPts[i] + t;
      bool ok = Xc.z() > 0 && (project(R, t, K, worldPts[i]) - imgPts[i]).norm() < pixelThreshold;
      mask[i] = ok ? 1 : 0;
      c += ok;
    }
    return c;
  };

  // If we have a pose prior, seed RANSAC's best with it (refined).
  if (hasGuess) {
    Matrix3d R = Rinit; Vector3d t = tinit;
    refineGN(worldPts, imgPts, K, R, t, 10);
    std::vector<char> mask(n, 0);
    int c = countInliers(R, t, mask);
    bestR = R; bestT = t; bestInliers = c; bestMask = mask;
  }

  std::vector<Vector3d> sX(6);
  std::vector<Vector2d> sxn(6);
  for (int it = 0; it < iterations; ++it) {
    int idx[6];
    for (int k = 0; k < 6; ++k) {
      bool dup; do { idx[k] = rng.below(n); dup = false; for (int j = 0; j < k; ++j) dup |= idx[j] == idx[k]; } while (dup);
      sX[k] = worldPts[idx[k]]; sxn[k] = xn[idx[k]];
    }
    Matrix3d R; Vector3d t;
    if (!dltPnP(sX, sxn, R, t)) continue;
    refineGN(worldPts, imgPts, K, R, t, 3);  // a few iters to settle the hypothesis
    std::vector<char> mask(n, 0);
    int c = countInliers(R, t, mask);
    if (c > bestInliers) { bestInliers = c; bestR = R; bestT = t; bestMask = mask; }
  }

  if (bestInliers < 6) return res;

  // Final refinement on all inliers.
  std::vector<Vector3d> iX; std::vector<Vector2d> iuv;
  for (int i = 0; i < n; ++i) if (bestMask[i]) { iX.push_back(worldPts[i]); iuv.push_back(imgPts[i]); }
  refineGN(iX, iuv, K, bestR, bestT, 15);
  bestInliers = countInliers(bestR, bestT, bestMask);

  res.ok = true;
  res.R = bestR;
  res.t = bestT;
  res.inliers = bestInliers;
  res.inlierMask = bestMask;
  return res;
}

}  // namespace webslam
