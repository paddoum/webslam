#include "two_view.h"

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

namespace webslam {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

// Tiny deterministic PRNG (xorshift) so RANSAC is reproducible without a
// global RNG — matters for WASM determinism and workflow resume.
struct Rng {
  uint64_t s;
  explicit Rng(unsigned seed) : s(seed ? seed : 0x9e3779b9u) {}
  uint32_t next() {
    s ^= s << 13; s ^= s >> 7; s ^= s << 17;
    return static_cast<uint32_t>(s);
  }
  int below(int n) { return static_cast<int>(next() % static_cast<uint32_t>(n)); }
};

Vector2d toNormalized(const Vector2d& px, const Intrinsics& K) {
  return Vector2d((px.x() - K.cx) / K.fx, (px.y() - K.cy) / K.fy);
}

// Normalized 8-point essential matrix from >=8 normalized correspondences.
// Applies Hartley isotropic normalization for conditioning.
Matrix3d eightPoint(const std::vector<Vector2d>& a, const std::vector<Vector2d>& b) {
  const int n = static_cast<int>(a.size());

  auto normalize = [](const std::vector<Vector2d>& pts, Matrix3d& T) {
    Vector2d mean = Vector2d::Zero();
    for (auto& p : pts) mean += p;
    mean /= pts.size();
    double meanDist = 0.0;
    for (auto& p : pts) meanDist += (p - mean).norm();
    meanDist /= pts.size();
    const double s = (meanDist > 1e-12) ? std::sqrt(2.0) / meanDist : 1.0;
    T << s, 0, -s * mean.x(),
         0, s, -s * mean.y(),
         0, 0, 1;
    std::vector<Vector2d> out(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
      out[i] = Vector2d(s * (pts[i].x() - mean.x()), s * (pts[i].y() - mean.y()));
    }
    return out;
  };

  Matrix3d Ta, Tb;
  auto an = normalize(a, Ta);
  auto bn = normalize(b, Tb);

  MatrixXd A(n, 9);
  for (int i = 0; i < n; ++i) {
    const double x1 = an[i].x(), y1 = an[i].y();
    const double x2 = bn[i].x(), y2 = bn[i].y();
    A.row(i) << x2 * x1, x2 * y1, x2, y2 * x1, y2 * y1, y2, x1, y1, 1.0;
  }

  Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeFullV);
  Eigen::VectorXd e = svd.matrixV().col(8);
  Matrix3d E;
  E << e(0), e(1), e(2), e(3), e(4), e(5), e(6), e(7), e(8);

  // Un-normalize, then enforce the essential constraint (singular values 1,1,0).
  E = Tb.transpose() * E * Ta;
  Eigen::JacobiSVD<Matrix3d> esvd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vector3d s(1, 1, 0);
  E = esvd.matrixU() * s.asDiagonal() * esvd.matrixV().transpose();
  return E;
}

// Sampson distance: a first-order geometric approximation of reprojection
// error for the epipolar constraint, in normalized coords.
double sampson(const Matrix3d& E, const Vector2d& a, const Vector2d& b) {
  Vector3d x1(a.x(), a.y(), 1.0), x2(b.x(), b.y(), 1.0);
  Vector3d Ex1 = E * x1;
  Vector3d Etx2 = E.transpose() * x2;
  const double num = x2.dot(Ex1);
  const double den = Ex1.x() * Ex1.x() + Ex1.y() * Ex1.y() +
                     Etx2.x() * Etx2.x() + Etx2.y() * Etx2.y();
  if (den < 1e-12) return 1e9;
  return (num * num) / den;
}

}  // namespace

Vector3d triangulate(const Vector2d& xn1, const Vector2d& xn2,
                     const Matrix3d& R, const Vector3d& t) {
  // P1 = [I|0], P2 = [R|t]. Build the 4x4 DLT system A X = 0.
  Eigen::Matrix<double, 3, 4> P1 = Eigen::Matrix<double, 3, 4>::Zero();
  P1.block<3, 3>(0, 0) = Matrix3d::Identity();
  Eigen::Matrix<double, 3, 4> P2;
  P2.block<3, 3>(0, 0) = R;
  P2.col(3) = t;

  Eigen::Matrix4d A;
  A.row(0) = xn1.x() * P1.row(2) - P1.row(0);
  A.row(1) = xn1.y() * P1.row(2) - P1.row(1);
  A.row(2) = xn2.x() * P2.row(2) - P2.row(0);
  A.row(3) = xn2.y() * P2.row(2) - P2.row(1);

  Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
  Eigen::Vector4d X = svd.matrixV().col(3);
  return X.head<3>() / X(3);
}

RelativePose estimateRelativePose(const std::vector<Match2D>& matches,
                                  const Intrinsics& K, double pixelThreshold,
                                  int iterations, unsigned seed) {
  RelativePose result;
  const int n = static_cast<int>(matches.size());
  if (n < 8) return result;

  // Normalize all correspondences to camera coordinates once.
  std::vector<Vector2d> na(n), nb(n);
  for (int i = 0; i < n; ++i) {
    na[i] = toNormalized(matches[i].a, K);
    nb[i] = toNormalized(matches[i].b, K);
  }

  // Threshold in normalized units (Sampson is squared distance).
  const double thNorm = pixelThreshold / std::max(K.fx, K.fy);
  const double thSq = thNorm * thNorm;

  Rng rng(seed);
  Matrix3d bestE = Matrix3d::Zero();
  int bestInliers = 0;
  std::vector<char> bestMask(n, 0);

  std::vector<Vector2d> sa(8), sb(8);
  for (int it = 0; it < iterations; ++it) {
    // Sample 8 distinct correspondences.
    int idx[8];
    for (int k = 0; k < 8; ++k) {
      bool dup;
      do {
        idx[k] = rng.below(n);
        dup = false;
        for (int j = 0; j < k; ++j) dup |= (idx[j] == idx[k]);
      } while (dup);
      sa[k] = na[idx[k]];
      sb[k] = nb[idx[k]];
    }

    Matrix3d E = eightPoint(sa, sb);

    int inliers = 0;
    std::vector<char> mask(n, 0);
    for (int i = 0; i < n; ++i) {
      if (sampson(E, na[i], nb[i]) < thSq) { mask[i] = 1; ++inliers; }
    }
    if (inliers > bestInliers) {
      bestInliers = inliers;
      bestE = E;
      bestMask = mask;
    }
  }

  if (bestInliers < 8) return result;

  // Refit E on all inliers for a better estimate.
  std::vector<Vector2d> ia, ib;
  ia.reserve(bestInliers); ib.reserve(bestInliers);
  for (int i = 0; i < n; ++i) if (bestMask[i]) { ia.push_back(na[i]); ib.push_back(nb[i]); }
  Matrix3d E = eightPoint(ia, ib);

  // Decompose E into the four candidate (R, t) solutions.
  Eigen::JacobiSVD<Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3d U = svd.matrixU();
  Matrix3d V = svd.matrixV();
  if (U.determinant() < 0) U.col(2) *= -1;
  if (V.determinant() < 0) V.col(2) *= -1;
  Matrix3d W;
  W << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  Matrix3d R1 = U * W * V.transpose();
  Matrix3d R2 = U * W.transpose() * V.transpose();
  Vector3d t = U.col(2);

  struct Cand { Matrix3d R; Vector3d t; };
  Cand cands[4] = {{R1, t}, {R1, -t}, {R2, t}, {R2, -t}};

  int bestFront = -1, bestIdx = 0;
  for (int c = 0; c < 4; ++c) {
    int front = 0;
    for (int i = 0; i < n; ++i) {
      if (!bestMask[i]) continue;
      Vector3d X = triangulate(na[i], nb[i], cands[c].R, cands[c].t);
      const double z1 = X.z();
      const double z2 = (cands[c].R * X + cands[c].t).z();
      if (z1 > 0 && z2 > 0) ++front;
    }
    if (front > bestFront) { bestFront = front; bestIdx = c; }
  }

  result.ok = true;
  result.R = cands[bestIdx].R;
  result.t = cands[bestIdx].t.normalized();
  result.inliers = bestInliers;
  result.inlierMask = bestMask;
  return result;
}

}  // namespace webslam
