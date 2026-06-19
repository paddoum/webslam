// Verifies the sliding-window VI optimizer, and its headline value: bridging a
// visual DROPOUT with the IMU. Synthesize a metric trajectory + IMU + noisy
// visual poses, blank the visual poses for two middle keyframes (a dropout),
// initialize those badly, and confirm the optimizer recovers them via the IMU.
#include <cmath>
#include <cstdio>
#include <vector>

#include "../src/imu_preint.h"
#include "../src/vi_optimizer.h"

using namespace webslam;
using Eigen::Matrix3d;
using Eigen::Vector3d;

int main() {
  const Vector3d g(0, 0, -9.81);
  const Vector3d w0(0.3, -0.2, 0.35);
  const Vector3d bgTrue(0.01, -0.015, 0.012), baTrue(0.04, -0.03, 0.05);
  const Vector3d Amp(0.7, 0.5, 0.6), wf(1.6, 2.2, 1.9);
  auto P = [&](double t){ return Vector3d(Amp.x()*std::sin(wf.x()*t), Amp.y()*std::sin(wf.y()*t), Amp.z()*std::sin(wf.z()*t)); };
  auto Vl = [&](double t){ return Vector3d(Amp.x()*wf.x()*std::cos(wf.x()*t), Amp.y()*wf.y()*std::cos(wf.y()*t), Amp.z()*wf.z()*std::cos(wf.z()*t)); };
  auto Ac = [&](double t){ return Vector3d(-Amp.x()*wf.x()*wf.x()*std::sin(wf.x()*t), -Amp.y()*wf.y()*wf.y()*std::sin(wf.y()*t), -Amp.z()*wf.z()*wf.z()*std::sin(wf.z()*t)); };
  auto Rt = [&](double t){ return expSO3(w0 * t); };

  const int nkf = 8; const double kfDt = 0.15, imuDt = 1.0/200;
  unsigned s = 7; auto noise=[&](double a){ s^=s<<13;s^=s>>17;s^=s<<5; return ((s%10000)/10000.0-0.5)*a; };

  std::vector<Preintegrated> pre;
  std::vector<VisualPose> vis(nkf);
  std::vector<ViState> truth(nkf), init(nkf);
  const int dropA = 4, dropB = 5;   // keyframes with NO visual measurement

  for (int i = 0; i < nkf; ++i) {
    const double t = i * kfDt;
    truth[i].R = Rt(t); truth[i].p = P(t); truth[i].v = Vl(t); truth[i].bg = bgTrue; truth[i].ba = baTrue;
    // visual measurement: true pose + small noise, except during the dropout
    vis[i].valid = (i != dropA && i != dropB);
    vis[i].R = Rt(t) * expSO3(Vector3d(noise(0.01), noise(0.01), noise(0.01)));
    vis[i].p = P(t) + Vector3d(noise(0.01), noise(0.01), noise(0.01));
    // initial guess: visual where valid; for dropout KFs, a deliberately wrong guess
    init[i].R = vis[i].valid ? vis[i].R : Rt((i-2)*kfDt);
    init[i].p = vis[i].valid ? vis[i].p : P((i-2)*kfDt) + Vector3d(0.3,0.2,0.1);
    init[i].v = Vl(t) + Vector3d(noise(0.3), noise(0.3), noise(0.3));
    init[i].bg = Vector3d::Zero(); init[i].ba = Vector3d::Zero();  // biases unknown at start
    if (i < nkf-1) {
      std::vector<ImuSample> smp;
      for (double tt=t; tt<t+kfDt-1e-9; tt+=imuDt){ ImuSample u; u.dt=imuDt; u.gyro=w0+bgTrue; u.acc=Rt(tt).transpose()*(Ac(tt)-g)+baTrue; smp.push_back(u); }
      pre.push_back(preintegrate(smp, Vector3d::Zero(), Vector3d::Zero()));  // raw (biases estimated by the optimizer)
    }
  }

  auto posErr = [&](const std::vector<ViState>& st, int i){ return (st[i].p - truth[i].p).norm(); };
  const double dropErrBefore = posErr(init, dropA) + posErr(init, dropB);

  std::vector<ViState> st = init;
  ViOptStats stats = viOptimize(st, pre, vis, g, 30);

  const double dropErrAfter = posErr(st, dropA) + posErr(st, dropB);
  double maxPos = 0, maxVel = 0; for (int i=0;i<nkf;i++){ maxPos=std::max(maxPos,posErr(st,i)); maxVel=std::max(maxVel,(st[i].v-truth[i].v).norm()); }

  printf("cost %.4f -> %.4f (%d iters)\n", stats.initCost, stats.finalCost, stats.iterations);
  printf("dropout KF pos err: %.3f m -> %.3f m\n", dropErrBefore, dropErrAfter);
  printf("max pos err %.3f m  max vel err %.3f m/s\n", maxPos, maxVel);

  const bool pass = stats.finalCost < stats.initCost &&
                    dropErrBefore > 0.2 &&        // the dropout KFs started badly wrong
                    dropErrAfter < 0.06 &&        // …and the IMU bridged them
                    maxPos < 0.06 && maxVel < 0.15;
  printf("%s\n", pass ? "PASS (VI optimizer — IMU bridges dropout)" : "FAIL (VI optimizer)");
  return pass ? 0 : 1;
}
