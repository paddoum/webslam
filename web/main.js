// webslam — camera/synthetic source → WASM visual-inertial SLAM.
import createSlamModule from './wasm/slam.js';
import { BUILD } from './version.js';

const PROC_W = 320, PROC_H = 240;       // 4:3 processing buffer
const MAX_FEATURES = 1200;              // more keypoints -> denser map, more robust matching
// Intrinsics for the 320x240 processing frame. Horizontal FOV ~63 deg is a
// reasonable approximation for an iPhone rear (wide) camera at 4:3.
const HFOV_DEG = 63;
const FX = (PROC_W / 2) / Math.tan((HFOV_DEG * Math.PI / 180) / 2);
const FY = FX, CX = PROC_W / 2, CY = PROC_H / 2;

const video = document.getElementById('video');
const overlay = document.getElementById('overlay');
const octx = overlay.getContext('2d');
const threshEl = document.getElementById('thresh');
const modeBtn = document.getElementById('modeBtn');
const el = (id) => document.getElementById(id);

// Stamp the build date/time into the header so you can confirm the running version.
{ const b = document.getElementById('build'); if (b) b.textContent = `build M14 · ${BUILD}`; }

const proc = document.createElement('canvas');
proc.width = PROC_W; proc.height = PROC_H;
const pctx = proc.getContext('2d', { willReadFrequently: true });

let Module = null, engine = null;
let mode = 'synthetic';
let frameIdx = 0;
const STATE = ['init', 'tracking', 'lost'];

// M5 — IMU. Latest device linear acceleration (m/s^2, gravity removed).
// M9.1 — monocular depth (async, in a Web Worker).
let depthOn = false, depthWorker = null, depthReady = false, depthBusy = false;
let lastDepthSent = 0, latestDepth = null, depthDirty = false;
let depthSendPose = null;      // pose snapshot at the instant a frame is sent to the worker
let depthAlign = null;         // {a,b,n,r}: invDepth ≈ a·(net/255) + b  (M9.2)
let lastDensified = 0;         // points added by the last depth densification (M9.3)
let fastThresh = 16;           // adaptive FAST threshold (auto-tuned to the feature target)
let gyroMag = 0;               // latest gyro angular speed (rad/s) for fast-motion search widening
let gyroVec = [0, 0, 0];       // latest gyro angular velocity vector (rad/s, device frame)
let lastFrameMs = 0;           // for per-frame dt

// --- Visual-inertial fusion / IMU coasting (keeps a pose when SLAM is lost) ---
let fusedR = null, fusedt = null;   // fused world->camera pose (flat 9 / 3)
let prevSlamR = null;               // previous tracked SLAM rotation (for ω_cam)
let coasting = false;               // true while propagating on the gyro
let gyroAid = true;                 // M13: feed the gyro rotation prior to the tracker
// Device→camera rotation: default phone remap, refined by calibration below.
let Rci = [1, 0, 0, 0, -1, 0, 0, 0, -1];
let RciCalibrated = false;
const gyroCal = { samples: 0, best: -1 };  // 24-candidate alignment accumulators
let anchorWorld = null;        // 3D world point where the AR sphere is anchored
let sphereRadiusWorld = 0.15;  // set at placement, scaled to the scene depth
let accel = { has: false, x: 0, y: 0, z: 0 };
let motionOn = false;
let motionEvents = 0;          // count of devicemotion events received
let scaleFed = 0;              // count of samples fed to the scale estimator
const SYN_TRUE_SCALE = 0.05;   // synthetic self-test ground truth (m per unit)
let startMs = 0;

// ─── Recording / replay ───────────────────────────────────────────────────────
let recording = false, recStartMs = 0;
let recFrames = [];   // [{tMs, y: Uint8Array}]  raw grayscale, captured sync
let recImu   = [];   // [{tMs, gx,gy,gz,ax,ay,az}]  device-frame
let replayActive = false;

// ---- synthetic 3D scene (cloud of points, moving virtual camera) ----
const scene = (() => {
  const pts = [];
  let s = 99887766;
  const rnd = () => { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return ((s >>> 0) % 100000) / 100000; };
  for (let i = 0; i < 200; i++) {
    pts.push({ X: [(rnd() - 0.5) * 8, (rnd() - 0.5) * 6, 4 + rnd() * 10],
               v: 70 + Math.floor(rnd() * 170), r: 1 + Math.floor(rnd() * 2) });
  }
  return pts;
})();

function cameraPose(f) {
  // Slow motion so the scene's appearance changes gradually. Sweeps in BOTH
  // yaw (phi) and pitch (theta) so the scan-coverage demo exercises both axes.
  // Yaw-only sweep (stable for the fragile synthetic tracker). The synthetic
  // camera only pans, so the scan bar fills the yaw axis but the guidance
  // correctly asks to "tilt up/down" — demonstrating the both-axes requirement.
  // On a real phone the user supplies the tilt to complete coverage.
  const phi = Math.sin(f * 0.008) * 0.35;
  const c = Math.cos(phi), sn = Math.sin(phi);
  const Rcw = [[c, 0, sn], [0, 1, 0], [-sn, 0, c]];
  const C = [Math.sin(f * 0.008) * 2.0, 0, Math.sin(f * 0.004) * 0.9];
  return { Rcw, C };
}
function mul3(R, v) {
  return [R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
          R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
          R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2]];
}
// Quaternion (qw,qx,qy,qz) from a row-major 3x3 rotation [r0..r8].
function matToQuat(m) {
  const tr = m[0] + m[4] + m[8];
  let qw, qx, qy, qz;
  if (tr > 0) { const S = Math.sqrt(tr + 1) * 2; qw = 0.25 * S; qx = (m[7]-m[5])/S; qy = (m[2]-m[6])/S; qz = (m[3]-m[1])/S; }
  else if (m[0] > m[4] && m[0] > m[8]) { const S = Math.sqrt(1+m[0]-m[4]-m[8])*2; qw = (m[7]-m[5])/S; qx = 0.25*S; qy = (m[1]+m[3])/S; qz = (m[2]+m[6])/S; }
  else if (m[4] > m[8]) { const S = Math.sqrt(1+m[4]-m[0]-m[8])*2; qw = (m[2]-m[6])/S; qx = (m[1]+m[3])/S; qy = 0.25*S; qz = (m[5]+m[7])/S; }
  else { const S = Math.sqrt(1+m[8]-m[0]-m[4])*2; qw = (m[3]-m[1])/S; qx = (m[2]+m[6])/S; qy = (m[5]+m[7])/S; qz = 0.25*S; }
  return [qw, qx, qy, qz];
}

// Project a world point with the current pose. Returns {u, v, depth} (camera px).
function projectWorld(X, R, t) {
  const xc = R[0]*X[0] + R[1]*X[1] + R[2]*X[2] + t[0];
  const yc = R[3]*X[0] + R[4]*X[1] + R[5]*X[2] + t[1];
  const zc = R[6]*X[0] + R[7]*X[1] + R[8]*X[2] + t[2];
  return { u: FX*xc/zc + CX, v: FY*yc/zc + CY, depth: zc };
}

// Place the AR anchor: pick the map point whose current projection is nearest
// the tapped pixel; fall back to a ray at the median map depth if none is close.
function placeAnchor(px, py) {
  if (!engine.mapTracked()) { el('status').textContent = 'wait for tracking before placing a sphere'; return; }
  const R = engine.poseR(), t = engine.poseT();
  const mp = engine.mapPoints();
  let best = 1e9, bestXYZ = null;
  const depths = [];
  for (let i = 0; i < mp.length; i += 3) {
    const X = [mp[i], mp[i+1], mp[i+2]];
    const p = projectWorld(X, R, t);
    if (p.depth <= 0.1) continue;
    depths.push(p.depth);
    const d2 = (p.u - px)*(p.u - px) + (p.v - py)*(p.v - py);
    if (d2 < best) { best = d2; bestXYZ = X; }
  }
  // Size the sphere relative to the scene scale (the SLAM gauge is arbitrary),
  // so it's visible and shrinks/grows correctly with distance.
  const setRadius = (X) => {
    const p = projectWorld(X, R, t);
    sphereRadiusWorld = Math.max(0.02, 0.05 * Math.abs(p.depth));
  };
  // M9.3: prefer depth — back-project the tapped pixel at its estimated depth.
  // Works anywhere (even textureless spots), but only when the depth↔map
  // alignment is trustworthy; otherwise fall back to map-point / ray placement.
  // |r| because the slope sign just encodes net-orientation (higher=closer vs
  // higher=farther). What matters for trust is the magnitude of correlation.
  const z = (depthAlign && Math.abs(depthAlign.spearman) > 0.5) ? metricDepthAt(px, py) : null;
  if (z != null && z > 0) {
    const dc = [(px - CX)/FX * z, (py - CY)/FY * z, z];  // camera-frame point
    const cx = -(R[0]*t[0]+R[3]*t[1]+R[6]*t[2]);
    const cy = -(R[1]*t[0]+R[4]*t[1]+R[7]*t[2]);
    const cz = -(R[2]*t[0]+R[5]*t[1]+R[8]*t[2]);
    // world = R^T (camPoint - t) = C + R^T·camPoint   (C = camera centre)
    anchorWorld = [
      cx + R[0]*dc[0]+R[3]*dc[1]+R[6]*dc[2],
      cy + R[1]*dc[0]+R[4]*dc[1]+R[7]*dc[2],
      cz + R[2]*dc[0]+R[5]*dc[1]+R[8]*dc[2],
    ];
    setRadius(anchorWorld);
  } else if (bestXYZ && best < 30*30) {
    anchorWorld = bestXYZ;                       // snap to real geometry
    setRadius(bestXYZ);
  } else if (depths.length) {
    depths.sort((a,b)=>a-b);
    const md = depths[depths.length >> 1];       // ray at median scene depth
    const dc = [(px - CX)/FX, (py - CY)/FY, 1];  // camera-frame ray
    // world dir = R^T * dc ; camera centre C = -R^T t
    const wdx = R[0]*dc[0]+R[3]*dc[1]+R[6]*dc[2];
    const wdy = R[1]*dc[0]+R[4]*dc[1]+R[7]*dc[2];
    const wdz = R[2]*dc[0]+R[5]*dc[1]+R[8]*dc[2];
    const cx = -(R[0]*t[0]+R[3]*t[1]+R[6]*t[2]);
    const cy = -(R[1]*t[0]+R[4]*t[1]+R[7]*t[2]);
    const cz = -(R[2]*t[0]+R[5]*t[1]+R[8]*t[2]);
    anchorWorld = [cx + md*wdx, cy + md*wdy, cz + md*wdz];
    setRadius(anchorWorld);
  }
}
// Scale self-test signal: a fast, clearly oscillatory (zero-mean acceleration)
// trajectory, decoupled from the slow SLAM camera. Returns position and its
// analytic acceleration so we can synthesize a clean IMU. Fast oscillation
// matters because the estimator removes the DC of acceleration (gravity/bias).
function scaleSelfTest(f) {
  const wx = 0.30, wy = 0.50, wz = 0.40;
  return {
    pos: [Math.sin(f * wx), 0.5 * Math.sin(f * wy + 1), 0.3 * Math.cos(f * wz)],
    acc: [-wx * wx * Math.sin(f * wx), -0.5 * wy * wy * Math.sin(f * wy + 1),
          -0.3 * wz * wz * Math.cos(f * wz)],
  };
}
function renderSynthetic(f) {
  pctx.fillStyle = '#0a0a14';
  pctx.fillRect(0, 0, PROC_W, PROC_H);
  const { Rcw, C } = cameraPose(f);
  for (const p of scene) {
    const Xc = mul3(Rcw, [p.X[0] - C[0], p.X[1] - C[1], p.X[2] - C[2]]);
    if (Xc[2] <= 0.5) continue;
    const px = FX * Xc[0] / Xc[2] + CX, py = FY * Xc[1] / Xc[2] + CY;
    if (px < 0 || py < 0 || px >= PROC_W || py >= PROC_H) continue;
    pctx.fillStyle = `rgb(${p.v},${p.v},${p.v})`;
    pctx.fillRect(px - p.r, py - p.r, p.r * 2 + 1, p.r * 2 + 1);
  }
}

// Draw a small top-down (X-Z) plot of the estimated camera trajectory.
function drawTrajectory(traj) {
  const w = 96, h = 72, ox = 6, oy = PROC_H - h - 6;
  octx.fillStyle = 'rgba(0,0,0,0.45)';
  octx.fillRect(ox, oy, w, h);
  octx.strokeStyle = 'rgba(255,255,255,0.2)';
  octx.strokeRect(ox, oy, w, h);
  if (traj.length < 6) return;
  let minx = 1e9, maxx = -1e9, minz = 1e9, maxz = -1e9;
  for (let i = 0; i < traj.length; i += 3) {
    minx = Math.min(minx, traj[i]); maxx = Math.max(maxx, traj[i]);
    minz = Math.min(minz, traj[i + 2]); maxz = Math.max(maxz, traj[i + 2]);
  }
  const sx = (w - 10) / Math.max(1e-3, maxx - minx), sz = (h - 10) / Math.max(1e-3, maxz - minz);
  const sc = Math.min(sx, sz);
  octx.strokeStyle = '#ffcc44';
  octx.beginPath();
  for (let i = 0; i < traj.length; i += 3) {
    const x = ox + 5 + (traj[i] - minx) * sc, y = oy + 5 + (traj[i + 2] - minz) * sc;
    i === 0 ? octx.moveTo(x, y) : octx.lineTo(x, y);
  }
  octx.stroke();
}

async function main() {
  Module = await createSlamModule();
  engine = new Module.SlamEngine(PROC_W, PROC_H, MAX_FEATURES);
  engine.setIntrinsics(FX, FY, CX, CY);
  engine.enableMapping();   // M3
  startMs = performance.now();
  el('status').textContent = 'WASM ready · synthetic scene (scale self-test running). Use camera + enable motion on a phone.';
  overlay.width = PROC_W; overlay.height = PROC_H;
  requestAnimationFrame(loop);
}

// Draw the camera frame into the 4:3 processing buffer with a CENTER CROP that
// matches the displayed video's object-fit:cover — so the image the SLAM sees
// is undistorted and the overlay aligns with what the user sees.
function drawCameraCropped() {
  const vw = video.videoWidth, vh = video.videoHeight;
  if (!vw || !vh) { pctx.fillStyle = '#000'; pctx.fillRect(0, 0, PROC_W, PROC_H); return; }
  const targetAR = PROC_W / PROC_H;            // 4:3
  let sw = vw, sh = vh, sx = 0, sy = 0;
  if (vw / vh > targetAR) { sw = vh * targetAR; sx = (vw - sw) / 2; }  // crop sides
  else { sh = vw / targetAR; sy = (vh - sh) / 2; }                     // crop top/bottom
  pctx.drawImage(video, sx, sy, sw, sh, 0, 0, PROC_W, PROC_H);
}

// Lazily spawn the depth worker (Transformers.js). Fails gracefully if the
// model can't load (e.g. no network) — SLAM keeps running regardless.
function ensureDepthWorker() {
  if (depthWorker) return;
  try {
    depthWorker = new Worker('./depth-worker.js', { type: 'module' });
    depthWorker.onmessage = (e) => {
      const m = e.data;
      if (m.type === 'ready') { depthReady = true; el('status').textContent = 'depth model ready'; }
      else if (m.type === 'progress') {
        const p = m.data;
        if (p && p.status === 'progress' && p.progress != null)
          el('status').textContent = `downloading depth model… ${Math.round(p.progress)}% (${p.file || ''})`;
        else if (p && p.status && p.status !== 'progress')
          el('status').textContent = 'depth model: ' + p.status;
      }
      else if (m.type === 'depth') { latestDepth = { w: m.width, h: m.height, c: m.channels || 1, data: new Uint8Array(m.data), pose: depthSendPose }; depthBusy = false; depthDirty = true; }
      else if (m.type === 'error') { el('status').textContent = 'depth: ' + m.message; depthOn = false; el('depthBtn').textContent = 'depth: off'; }
    };
    depthWorker.onerror = (e) => { el('status').textContent = 'depth worker error'; depthOn = false; el('depthBtn').textContent = 'depth: off'; };
    el('status').textContent = 'loading depth model… (first time downloads ~25 MB)';
    // Watchdog: if it never reports ready, say so instead of spinning forever.
    setTimeout(() => {
      if (depthOn && !depthReady)
        el('status').textContent = 'depth model still loading — slow network or blocked. Try reload / Wi-Fi.';
    }, 60000);
  } catch (e) { el('status').textContent = 'depth unavailable: ' + e.message; depthOn = false; }
}

// Map a normalized value [0,1] to a simple blue→cyan→yellow→red heatmap.
function heat(t) {
  t = Math.max(0, Math.min(1, t));
  const r = Math.round(255 * Math.max(0, Math.min(1, 1.5 - Math.abs(4*t - 3))));
  const g = Math.round(255 * Math.max(0, Math.min(1, 1.5 - Math.abs(4*t - 2))));
  const b = Math.round(255 * Math.max(0, Math.min(1, 1.5 - Math.abs(4*t - 1))));
  return [r, g, b];
}

// Network depth at a proc-frame pixel (0..255). Optionally takes a 3x3 median
// window — depth nets produce sharp artifacts at object edges (where corners
// often sit), so a tiny robust window gives a much more reliable sample.
function depthAt(u, v, robust = false) {
  if (!latestDepth) return null;
  const { w, h, c, data } = latestDepth;
  const x = Math.round(u / PROC_W * w), y = Math.round(v / PROC_H * h);
  if (x < 1 || y < 1 || x >= w - 1 || y >= h - 1) {
    if (x < 0 || y < 0 || x >= w || y >= h) return null;
    return data[(y * w + x) * c];
  }
  if (!robust) return data[(y * w + x) * c];
  const samples = [];
  for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++)
    samples.push(data[((y + dy) * w + (x + dx)) * c]);
  samples.sort((a, b) => a - b);
  return samples[4];  // median of 9
}

// M9.2: fit invDepth ≈ a·(net/255) + b using the visible map points as anchors,
// so the network's relative depth becomes map-consistent metric inverse depth.
function computeDepthAlignment() {
  // Align using the pose the depth frame was captured at (not the current one),
  // so map-point projections match the depth map's viewpoint.
  if (!latestDepth || !latestDepth.pose) { depthAlign = null; return; }
  const R = latestDepth.pose.R, t = latestDepth.pose.t;
  const mp = engine.mapPoints();

  // Collect (net inverse-depth, map inverse-depth) anchor pairs.
  let pts = [];  // {x: net/255, y: 1/depth}
  let nMin = 1e9, nMax = -1e9;
  for (let i = 0; i < mp.length; i += 3) {
    const p = projectWorld([mp[i], mp[i+1], mp[i+2]], R, t);
    if (p.depth <= 0.05 || p.u < 2 || p.v < 2 || p.u >= PROC_W - 2 || p.v >= PROC_H - 2) continue;
    const nd = depthAt(p.u, p.v, /*robust=*/true);
    if (nd == null) continue;
    pts.push({ x: nd / 255, y: 1 / p.depth });
    if (nd < nMin) nMin = nd; if (nd > nMax) nMax = nd;
  }
  if (pts.length < 12) { depthAlign = null; return; }

  // Pearson correlation + LS line over a set.
  const fit = (arr) => {
    let n = arr.length, sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
    for (const p of arr) { sx += p.x; sy += p.y; sxx += p.x*p.x; sxy += p.x*p.y; syy += p.y*p.y; }
    const denom = n*sxx - sx*sx;
    const a = Math.abs(denom) < 1e-9 ? 0 : (n*sxy - sx*sy) / denom;
    const b = (sy - a*sx) / n;
    const r = (n*sxy - sx*sy) / Math.sqrt(denom * (n*syy - sy*sy) + 1e-12);
    return { a, b, r };
  };

  // Robust: fit, drop worst 40% by residual, refit (kills triangulation outliers).
  let f = fit(pts);
  const resid = pts.map(p => ({ p, e: Math.abs(p.y - (f.a*p.x + f.b)) }));
  resid.sort((u, v) => u.e - v.e);
  const inliers = resid.slice(0, Math.max(12, Math.floor(pts.length * 0.6))).map(o => o.p);
  const fr = fit(inliers);

  // Spearman rank correlation (scale-free, robust to monotonic nonlinearity):
  // distinguishes "nonlinear/outliers" (high Spearman) from "spatial bug" (low).
  const rank = (key) => {
    const idx = pts.map((p, i) => i).sort((i, j) => pts[i][key] - pts[j][key]);
    const rk = new Array(pts.length);
    for (let k = 0; k < idx.length; k++) rk[idx[k]] = k;
    return rk;
  };
  const rx = rank('x'), ry = rank('y');
  let dd = 0; for (let i = 0; i < pts.length; i++) { const d = rx[i] - ry[i]; dd += d*d; }
  const sp = 1 - (6 * dd) / (pts.length * (pts.length*pts.length - 1));

  depthAlign = { a: fr.a, b: fr.b, n: pts.length, r: f.r, rRobust: fr.r, spearman: sp, netSpread: nMax - nMin };
}

// Aligned metric depth (map units) at a proc pixel, or null.
function metricDepthAt(u, v) {
  if (!depthAlign) return null;
  const nd = depthAt(u, v);
  if (nd == null) return null;
  const inv = depthAlign.a * (nd / 255) + depthAlign.b;
  if (inv <= 1e-4) return null;
  return 1 / inv;
}

// --- Small flat-row-major 3x3 / SO(3) helpers for the fusion layer ---
function m3mul(A, B) {
  const C = new Array(9);
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++)
    C[r*3+c] = A[r*3]*B[c] + A[r*3+1]*B[3+c] + A[r*3+2]*B[6+c];
  return C;
}
function m3T(A) { return [A[0],A[3],A[6], A[1],A[4],A[7], A[2],A[5],A[8]]; }
function m3v(A, v) { return [A[0]*v[0]+A[1]*v[1]+A[2]*v[2], A[3]*v[0]+A[4]*v[1]+A[5]*v[2], A[6]*v[0]+A[7]*v[1]+A[8]*v[2]]; }
function expSO3(w) {  // axis-angle vector -> rotation (Rodrigues)
  const a = Math.hypot(w[0], w[1], w[2]);
  if (a < 1e-8) return [1,0,0, 0,1,0, 0,0,1];
  const x = w[0]/a, y = w[1]/a, z = w[2]/a, s = Math.sin(a), c = Math.cos(a), C = 1 - c;
  return [c+x*x*C, x*y*C-z*s, x*z*C+y*s,
          y*x*C+z*s, c+y*y*C, y*z*C-x*s,
          z*x*C-y*s, z*y*C+x*s, c+z*z*C];
}
function logSO3(R) {  // rotation -> axis-angle vector
  const tr = (R[0]+R[4]+R[8]-1)/2, a = Math.acos(Math.max(-1, Math.min(1, tr)));
  if (a < 1e-6) return [0,0,0];
  const k = a / (2*Math.sin(a));
  return [(R[7]-R[5])*k, (R[2]-R[6])*k, (R[3]-R[1])*k];
}
// 24 signed axis-permutation rotations (device->camera candidates).
const RCI_CANDIDATES = (() => {
  const perms = [[0,1,2],[0,2,1],[1,0,2],[1,2,0],[2,0,1],[2,1,0]];
  const out = [];
  for (const p of perms) for (let s = 0; s < 8; s++) {
    const sg = [(s&1)?-1:1, (s&2)?-1:1, (s&4)?-1:1];
    const M = [0,0,0,0,0,0,0,0,0];
    M[0*3+p[0]] = sg[0]; M[1*3+p[1]] = sg[1]; M[2*3+p[2]] = sg[2];
    const det = M[0]*(M[4]*M[8]-M[5]*M[7]) - M[1]*(M[3]*M[8]-M[5]*M[6]) + M[2]*(M[3]*M[7]-M[4]*M[6]);
    if (Math.abs(det - 1) < 1e-6) out.push(M);
  }
  return out;  // 24
})();
const gyroScores = new Float64Array(RCI_CANDIDATES.length);

// Update the fused pose. Tracking → snap to SLAM (and calibrate Rci from the
// gyro vs SLAM rotation). Lost → dead-reckon rotation from the gyro, hold
// translation. This keeps a usable pose (and the sphere) at all times.
function updateFusion(dt, tracking) {
  if (tracking) {
    const R = Array.from(engine.poseR()), t = Array.from(engine.poseT());
    if (prevSlamR && dt > 0) {
      // SLAM camera body angular velocity: for R = world->cam,
      // R_t = exp(-[ω]·dt)·R_{t-1}  ⇒  ω = -logSO3(R_t·R_{t-1}ᵀ)/dt.
      const wc = logSO3(m3mul(R, m3T(prevSlamR))).map(x => -x / dt);
      const wcMag = Math.hypot(wc[0], wc[1], wc[2]), wdMag = Math.hypot(gyroVec[0], gyroVec[1], gyroVec[2]);
      if (wcMag > 0.3 && wdMag > 0.3) {  // calibrate only under real rotation
        for (let i = 0; i < RCI_CANDIDATES.length; i++) {
          const cw = m3v(RCI_CANDIDATES[i], gyroVec);
          gyroScores[i] += (cw[0]*wc[0] + cw[1]*wc[1] + cw[2]*wc[2]) / (wcMag * wdMag);  // cosine
        }
        gyroCal.samples++;
        if (gyroCal.samples >= 10) {
          let bi = 0; for (let i = 1; i < gyroScores.length; i++) if (gyroScores[i] > gyroScores[bi]) bi = i;
          // Lock only on a genuinely strong, consistent alignment.
          if (gyroScores[bi] / gyroCal.samples > 0.6) { Rci = RCI_CANDIDATES[bi]; RciCalibrated = true; gyroCal.best = bi; }
        }
      }
    }
    prevSlamR = R;
    fusedR = R; fusedt = t;
    coasting = false;
  } else if (fusedR) {
    // Coast: rotate the fused pose by the gyro increment, but ONLY if the
    // gyro→camera rotation is calibrated — otherwise hold (rotating with a
    // wrong Rci would fling the sphere the wrong way).
    if (RciCalibrated) {
      const wcam = m3v(Rci, gyroVec);  // camera-frame angular velocity
      fusedR = m3mul(expSO3([-wcam[0]*dt, -wcam[1]*dt, -wcam[2]*dt]), fusedR);
    }
    // translation held (accel double-integration drifts too fast to trust)
    prevSlamR = null;
    coasting = true;
  }
}

function step() {
  if (mode === 'synthetic') renderSynthetic(frameIdx);
  else if (mode === 'camera') drawCameraCropped();
  // mode === 'replay': proc canvas was pre-filled by the replay loop

  const img = pctx.getImageData(0, 0, PROC_W, PROC_H);
  engine.inputView().set(img.data);
  // Capture Y-plane for recording (sync — JPEG compression happens on stop).
  if (recording) {
    const d = img.data, Y = new Uint8Array(PROC_W * PROC_H);
    for (let i = 0; i < PROC_W * PROC_H; i++)
      Y[i] = (0.299 * d[i*4] + 0.587 * d[i*4+1] + 0.114 * d[i*4+2] + 0.5) | 0;
    recFrames.push({tMs: performance.now() - recStartMs, y: Y});
  }
  // Gyro-derived motion hint: expected feature shift this frame ≈ ω·dt·focal.
  const nowMs = performance.now();
  const dt = lastFrameMs ? Math.min(0.2, Math.max(0.005, (nowMs - lastFrameMs) / 1000)) : 0.033;
  lastFrameMs = nowMs;
  engine.setMotionHint(gyroMag * dt * FX);
  // Gyro rotation PRIOR (not just a search-radius hint): reorient the pose
  // prediction so fast turns / orbiting an object don't drop tracking, and so
  // the rotation keeps dead-reckoning while lost. Only when Rci is calibrated.
  // Toggleable so the benefit can be A/B'd on a real device (its value depends
  // on IMU latency + Rci accuracy, which simulation can't validate).
  if (gyroAid && RciCalibrated) {
    const wcam = m3v(Rci, gyroVec);  // camera-frame angular velocity (rad/s)
    engine.setGyroDelta(wcam[0], wcam[1], wcam[2], dt);
  }
  const n = engine.processFrame(PROC_W, PROC_H, fastThresh);
  // Adaptive detection: drive the FAST threshold toward the feature-count target
  // so low-texture scenes (e.g. a wood floor) still yield enough corners to track.
  const target = parseInt(threshEl.value, 10);
  if (n < target * 0.8 && fastThresh > 4) fastThresh -= 1;
  else if (n > target * 1.2 && fastThresh < 70) fastThresh += 1;

  octx.clearRect(0, 0, PROC_W, PROC_H);

  // Faint blue: this frame's detected corners.
  const kp = engine.keypoints();
  octx.fillStyle = 'rgba(120,160,255,0.4)';
  for (let i = 0; i < kp.length; i += 3) octx.fillRect(kp[i] - 1, kp[i + 1] - 1, 2, 2);

  // Update the fused pose (SLAM when tracking, gyro-coasted when lost).
  const tracking = engine.mapTracked();
  updateFusion(dt, tracking);
  const FR = fusedR, FT = fusedt;  // pose used for all world overlays

  // Green: the persistent 3D map reprojected with the FUSED pose. Drawn while
  // tracking AND while coasting (amber) so the overlay never blinks out.
  let drawn = 0;
  if (FR) {
    const mp = engine.mapPoints();
    octx.strokeStyle = coasting ? '#ffb020' : '#39ff88';   // amber = gyro-coasting
    octx.lineWidth = 1;
    for (let i = 0; i < mp.length; i += 3) {
      const X = [mp[i], mp[i + 1], mp[i + 2]];
      const Xc = [FR[0]*X[0]+FR[1]*X[1]+FR[2]*X[2]+FT[0],
                  FR[3]*X[0]+FR[4]*X[1]+FR[5]*X[2]+FT[1],
                  FR[6]*X[0]+FR[7]*X[1]+FR[8]*X[2]+FT[2]];
      if (Xc[2] <= 0.2) continue;
      const u = FX * Xc[0] / Xc[2] + CX, v = FY * Xc[1] / Xc[2] + CY;
      if (u < 0 || v < 0 || u >= PROC_W || v >= PROC_H) continue;
      octx.strokeRect(u - 2, v - 2, 4, 4);
      drawn++;
    }
  }

  // AR sphere: anchored at a fixed 3D world point, ALWAYS drawn (with the fused
  // pose) — so it stays put even when SLAM is lost and we're coasting on gyro.
  if (anchorWorld && FR) {
    const p = projectWorld(anchorWorld, FR, FT);
    if (p.depth > 0.1 && p.u > -40 && p.v > -40 && p.u < PROC_W + 40 && p.v < PROC_H + 40) {
      const r = Math.max(4, Math.min(90, FX * sphereRadiusWorld / p.depth));
      const g = octx.createRadialGradient(p.u - r*0.35, p.v - r*0.35, r*0.1, p.u, p.v, r);
      // dim slightly while coasting to signal it's gyro-only
      g.addColorStop(0, coasting ? '#ffcf90' : '#ffe0b0');
      g.addColorStop(0.5, coasting ? '#d4701f' : '#ff8c32');
      g.addColorStop(1, '#9c4000');
      octx.fillStyle = g;
      octx.beginPath(); octx.arc(p.u, p.v, r, 0, 2*Math.PI); octx.fill();
      octx.strokeStyle = 'rgba(0,0,0,0.45)'; octx.lineWidth = 1; octx.stroke();
    }
  }

  drawTrajectory(engine.trajectory());

  // --- M9.1: depth (async ~2 Hz in a worker) ---
  if (depthOn && depthReady && !depthBusy && (performance.now() - lastDepthSent) > 450) {
    const di = pctx.getImageData(0, 0, PROC_W, PROC_H);  // fresh RGBA for the model
    // Snapshot the pose NOW so the (async, ~2 s later) depth map is aligned with
    // the viewpoint it was actually captured from — not a moved pose.
    depthSendPose = tracking ? { R: Array.from(engine.poseR()), t: Array.from(engine.poseT()) } : null;
    depthWorker.postMessage({ type: 'frame', data: di.data.buffer, width: PROC_W, height: PROC_H }, [di.data.buffer]);
    depthBusy = true;
    lastDepthSent = performance.now();
  }
  // M9.2: when a fresh depth map arrives, align it to the SLAM map (fit the
  // affine that maps network inverse-depth to map inverse-depth).
  if (depthDirty) {
    computeDepthAlignment();
    // M9.3: densify the map from this depth frame — but only when it's well
    // aligned AND the camera is still near the viewpoint the depth was captured
    // at (so the depth matches the current corners). Else the points would be
    // wrong; tracking would self-cull them, but better not to inject them.
    if (depthAlign && Math.abs(depthAlign.spearman) > 0.5 && tracking && latestDepth.pose &&
        latestDepth.c === 1 && latestDepth.w === PROC_W && latestDepth.h === PROC_H) {
      const Rc = engine.poseR();
      let tr = 0; for (let k = 0; k < 9; k++) tr += Rc[k] * latestDepth.pose.R[k];
      const ang = Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2))) * 180 / Math.PI;
      if (ang < 10) {
        engine.depthView().set(latestDepth.data);
        lastDensified = engine.densifyFromDepth(latestDepth.w, latestDepth.h, depthAlign.a, depthAlign.b);
      }
    }
    depthDirty = false;
  }

  if (depthOn && latestDepth) {
    const { w, h, c, data } = latestDepth;
    const TW = 116, TH = 88, ox = PROC_W - TW - 6, oy = 6;
    const id = octx.createImageData(TW, TH);
    for (let ty = 0; ty < TH; ty++) for (let tx = 0; tx < TW; tx++) {
      const sx = (tx / TW * w) | 0, sy = (ty / TH * h) | 0;
      const t = data[(sy * w + sx) * c] / 255;
      const [r, g, b] = heat(t); const o = (ty * TW + tx) * 4;
      id.data[o] = r; id.data[o+1] = g; id.data[o+2] = b; id.data[o+3] = 255;
    }
    octx.putImageData(id, ox, oy);
    octx.strokeStyle = 'rgba(255,255,255,0.3)'; octx.strokeRect(ox, oy, TW, TH);
  }

  // --- M5: feed the metric-scale estimator ---
  if (mode === 'synthetic') {
    // Self-test: feed a known trajectory + a clean IMU derived from it (scaled
    // by SYN_TRUE_SCALE). The estimator should recover SYN_TRUE_SCALE.
    const stp = scaleSelfTest(frameIdx);  // world == camera here -> identity quaternion
    engine.addScaleSample(frameIdx, stp.pos[0], stp.pos[1], stp.pos[2], 1, 0, 0, 0,
                          SYN_TRUE_SCALE * stp.acc[0], SYN_TRUE_SCALE * stp.acc[1], SYN_TRUE_SCALE * stp.acc[2]);
  } else if (tracking && accel.has && engine.trackingInliers() >= 30) {
    // Only feed scale from well-tracked frames. Pass the camera centre, the
    // world->camera rotation (quaternion), and the RAW device-frame accel —
    // the estimator auto-calibrates the device->camera alignment.
    const R = engine.poseR(), t = engine.poseT();
    const cx = -(R[0]*t[0] + R[3]*t[1] + R[6]*t[2]);
    const cy = -(R[1]*t[0] + R[4]*t[1] + R[7]*t[2]);
    const cz = -(R[2]*t[0] + R[5]*t[1] + R[8]*t[2]);
    const q = matToQuat(R);
    engine.addScaleSample((performance.now() - startMs) / 1000, cx, cy, cz,
                          q[0], q[1], q[2], q[3], accel.x, accel.y, accel.z);
    scaleFed++;
  }

  // Explicit, un-emptyable status so it's clear WHY alignment isn't showing.
  const da = el('depthAlign');
  if (da) {
    let s;
    if (!depthOn) s = 'off';
    else if (!depthReady) s = 'loading model';
    else if (!latestDepth) s = 'no depth yet';
    else if (depthAlign) {
      // raw r | robust r | Spearman rank corr — the three together diagnose it
      s = `sp=${depthAlign.spearman.toFixed(2)} rob=${depthAlign.rRobust.toFixed(2)} +${lastDensified}pts`;
    }
    else if (!latestDepth.pose) s = 'need tracking';
    else s = 'too few pts';
    da.textContent = s;
  }

  el('kpCount').textContent = n;
  const st = engine.mapState();
  el('state').textContent = coasting
    ? (RciCalibrated ? 'coasting (gyro✓)' : 'coasting (gyro uncal — rotate while tracking)')
    : (STATE[st] || '?') + (st === 1 && !RciCalibrated ? ' · gyro calibrating…' : st === 1 ? ' · gyro✓' : '');
  el('mapPts').textContent = engine.mapNumPoints();
  el('kfs').textContent = engine.mapNumKeyframes();
  el('inliers').textContent = tracking ? engine.trackingInliers() : 0;

  // Metric scale readouts. (Camera-mode status is set below with diagnostics.) Show the RAW estimate once the window has data (even
  // below the validity threshold) so it's visible what the estimator is doing;
  // mark low-confidence values with "~".
  // M12.4: prefer the live VINS-Mono-style VI-init scale (a clean one-shot
  // linear solve over a keyframe window) when it has a plausible solution; it
  // also gives a gravity direction. Fall back to the M5/M8 correlation scale.
  const viOk = engine.viOk();
  const viS = engine.viScale();
  const sc = engine.metricScale();
  const conf = engine.scaleConfidence();
  const samples = engine.scaleSamples();
  const usedScale = viOk ? viS : sc;
  const showScale = viOk || (samples > 6 && sc > 0);
  if (showScale) {
    const tag = viOk ? 'VI ' : (engine.scaleValid() ? '' : '~');
    el('scale').textContent = mode === 'synthetic'
      ? usedScale.toFixed(4) + ` (true ${SYN_TRUE_SCALE})`
      : tag + usedScale.toFixed(4);
    el('scaleConf').textContent = (viOk ? engine.viConfidence() : conf).toFixed(2)
      + (viOk ? ` · ${engine.viKeyframes()}kf` : '');
    if (mode === 'camera') {
      const traj = engine.trajectory();
      let vlen = 0;
      for (let i = 3; i < traj.length; i += 3)
        vlen += Math.hypot(traj[i]-traj[i-3], traj[i+1]-traj[i-2], traj[i+2]-traj[i-1]);
      const meters = usedScale * vlen;
      const valid = viOk || engine.scaleValid();
      el('metricMove').textContent = (valid && meters < 100) ? meters.toFixed(2) + ' m' : '—';
    } else {
      el('metricMove').textContent = '(self-test)';
    }
  } else {
    el('scale').textContent = '—';
    el('scaleConf').textContent = '—';
    el('metricMove').textContent = mode === 'camera' && !motionOn ? 'enable motion' : '—';
  }

  // Camera-mode IMU diagnostics appended to the status line.
  if (mode === 'camera') {
    const amag = accel.has ? Math.hypot(accel.x, accel.y, accel.z) : 0;
    const base = st === 1 ? 'tracking' : st === 0 ? 'init — slide sideways' : 'lost — reset + slide';
    const viTag = viOk
      ? `vi✓ |g|${engine.viGravityMag().toFixed(1)} ${engine.viKeyframes()}kf`
      : `vi:${RciCalibrated ? engine.viKeyframes() + 'kf…' : 'cal'}`;
    el('status').textContent =
      `${base} · motion:${motionOn ? 'on' : 'OFF'} evt:${motionEvents} |a|:${amag.toFixed(1)} fed:${scaleFed} samp:${samples} ${viTag}`;
  }

  // --- Scan coverage (ARKit-style: requires BOTH pan and tilt) ---
  // Coverage is the angular span of viewing directions in yaw and pitch. The
  // bar fills only as you cover both axes, and "ready" needs both targets met.
  const YAW_TARGET = 30, PITCH_TARGET = 18;  // degrees of pan / tilt
  const yawDeg = engine.coverageYawDeg();
  const pitchDeg = engine.coveragePitchDeg();
  const yawFrac = Math.min(1, yawDeg / YAW_TARGET);
  const pitchFrac = Math.min(1, pitchDeg / PITCH_TARGET);
  const progress = (st === 0) ? 0 : 0.5 * yawFrac + 0.5 * pitchFrac;
  const ready = st === 1 && yawFrac >= 1 && pitchFrac >= 1;
  const pct = Math.round(progress * 100);
  el('scanPct').textContent = pct + '%';
  const bar = el('scanBar');
  bar.style.width = pct + '%';
  bar.classList.toggle('ready', ready);
  el('scanHint').textContent =
    st === 0 ? 'point at texture and slide sideways to start'
    : st === 2 ? 'lost — return to a mapped area to relocalize'
    : ready ? `well covered — pan ${yawDeg.toFixed(0)}° tilt ${pitchDeg.toFixed(0)}°`
    : (yawFrac < pitchFrac) ? `keep panning left/right (${yawDeg.toFixed(0)}°/${YAW_TARGET}°)`
    : `keep tilting up/down (${pitchDeg.toFixed(0)}°/${PITCH_TARGET}°)`;

  frameIdx++;
  return { n, state: engine.mapState(), pts: engine.mapNumPoints(), kfs: engine.mapNumKeyframes(),
           inliers: engine.trackingInliers(), tracked: engine.mapTracked(), reproj: drawn };
}

let lastFpsT = 0, frames = 0;
function loop(now) {
  step();
  frames++;
  if (now - lastFpsT > 500) { el('fps').textContent = Math.round((frames * 1000) / (now - lastFpsT)); frames = 0; lastFpsT = now; }
  requestAnimationFrame(loop);
}
window.__step = (k = 1) => { let r; for (let i = 0; i < k; i++) r = step(); return r; };

modeBtn.addEventListener('click', async () => {
  if (mode === 'synthetic') {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({
        video: { facingMode: 'environment', width: { ideal: 640 }, height: { ideal: 480 } }, audio: false });
      video.srcObject = stream; await video.play();
      mode = 'camera'; modeBtn.textContent = 'switch to synthetic';
      engine.enableMapping(); engine.resetScale(); scaleFed = 0;  // fresh map + scale window
      enableMotion();  // gyro aids fast-motion tracking (same gesture grants it)
      el('status').textContent = 'camera live · slide sideways to start a map';
    } catch (e) { el('status').textContent = 'camera unavailable (' + e.name + ') — staying on synthetic'; }
  } else {
    mode = 'synthetic'; modeBtn.textContent = 'switch to camera';
    engine.enableMapping(); engine.resetScale(); scaleFed = 0;
    el('status').textContent = 'synthetic 3D scene';
  }
});

// Reset the map (re-create a fresh SlamMap) so the user can re-initialize
// without reloading — useful after tracking is lost.
el('resetBtn').addEventListener('click', () => {
  engine.enableMapping();
  engine.resetScale();
  el('status').textContent = 'map reset · slide the phone sideways over texture to re-initialize';
});

// M5: device motion. iOS requires an explicit permission request from a user
// gesture, served over HTTPS.
function onMotion(e) {
  motionEvents++;
  // Gyro angular speed (rad/s) — drives the fast-motion search-window widening.
  const rr = e.rotationRate;
  if (rr) {
    const D = Math.PI / 180;
    // Device angular-velocity vector; the exact axis→camera mapping is resolved
    // by the Rci calibration, so any consistent ordering is fine.
    gyroVec = [(rr.beta || 0) * D, (rr.gamma || 0) * D, (rr.alpha || 0) * D];
    gyroMag = Math.hypot(gyroVec[0], gyroVec[1], gyroVec[2]);
  }
  const a = e.acceleration || e.accelerationIncludingGravity;  // prefer gravity-removed
  if (!a) return;
  accel.has = true;
  accel.x = a.x || 0; accel.y = a.y || 0; accel.z = a.z || 0;

  // M12.4: feed the live VI-init window with high-rate IMU in the CAMERA frame.
  // The window's preintegration needs specific force INCLUDING gravity (gravity
  // is solved for), so use accelerationIncludingGravity. We can only rotate into
  // the camera frame once the gyro→camera alignment (Rci) is calibrated.
  const aig = e.accelerationIncludingGravity;
  if (recording) {
    recImu.push({tMs: performance.now() - recStartMs,
      gx: gyroVec[0], gy: gyroVec[1], gz: gyroVec[2],
      ax: aig ? (aig.x || 0) : 0,
      ay: aig ? (aig.y || 0) : 0,
      az: aig ? (aig.z || 0) : 0});
  }
  if (engine && RciCalibrated && aig) {
    const nowMs = performance.now();
    let dt = e.interval || (lastMotionMs ? (nowMs - lastMotionMs) / 1000 : 0);
    lastMotionMs = nowMs;
    if (dt > 1e-4 && dt < 0.1) {
      const gcam = m3v(Rci, gyroVec);
      const acam = m3v(Rci, [aig.x || 0, aig.y || 0, aig.z || 0]);
      engine.addImuSample(dt, gcam[0], gcam[1], gcam[2], acam[0], acam[1], acam[2]);
    }
  }
}
let lastMotionMs = 0;
// Enable device motion (gyro + accel). Gyro now aids fast-motion tracking, not
// just metric scale — so we also try to enable it when the camera turns on.
async function enableMotion() {
  if (motionOn) return true;
  try {
    if (typeof DeviceMotionEvent !== 'undefined' &&
        typeof DeviceMotionEvent.requestPermission === 'function') {
      const res = await DeviceMotionEvent.requestPermission();  // iOS
      if (res !== 'granted') { el('status').textContent = 'motion permission denied'; return false; }
    }
    window.addEventListener('devicemotion', onMotion);
    motionOn = true;
    el('motionBtn').textContent = 'motion on';
    return true;
  } catch (err) { el('status').textContent = 'motion error: ' + err.name; return false; }
}
el('motionBtn').addEventListener('click', async () => {
  if (await enableMotion()) el('status').textContent = 'motion on · gyro aids fast tracking + recovers scale';
});

// Tap the view to drop the AR sphere at that spot; the button drops it at centre.
const stage = document.getElementById('stage');
stage.addEventListener('click', (e) => {
  const rect = stage.getBoundingClientRect();
  const px = (e.clientX - rect.left) / rect.width * PROC_W;
  const py = (e.clientY - rect.top) / rect.height * PROC_H;
  placeAnchor(px, py);
});
el('sphereBtn').addEventListener('click', () => placeAnchor(PROC_W / 2, PROC_H / 2));
el('depthBtn').addEventListener('click', () => {
  depthOn = !depthOn;
  el('depthBtn').textContent = depthOn ? 'depth: on' : 'depth: off';
  if (depthOn) ensureDepthWorker();
  else latestDepth = null;
});
el('gyroAidBtn').addEventListener('click', () => {
  gyroAid = !gyroAid;
  el('gyroAidBtn').textContent = gyroAid ? 'gyro-aid: on' : 'gyro-aid: off';
});
el('recBtn').addEventListener('click', () => {
  if (replayActive) return;
  if (recording) {
    stopAndSaveRecording();
  } else {
    recording = true; recStartMs = performance.now(); recFrames = []; recImu = [];
    el('recBtn').textContent = 'stop rec';
    el('status').textContent = 'recording… tap stop when done';
  }
});
el('replayBtn').addEventListener('click', () => { if (!recording) el('replayFileInput').click(); });
el('replayFileInput').addEventListener('change', async (e) => {
  const f = e.target.files[0];
  if (f) await loadAndReplay(f);
  e.target.value = '';
});
// ─── Recording: JPEG-encode captured frames and download as .wsrec ────────────
async function stopAndSaveRecording() {
  recording = false;
  el('recBtn').textContent = 'record';
  el('status').textContent = `encoding ${recFrames.length} frames…`;

  const oc = new OffscreenCanvas(PROC_W, PROC_H);
  const rc = oc.getContext('2d');
  const jpegBufs = [];
  for (let i = 0; i < recFrames.length; i++) {
    const {tMs, y} = recFrames[i];
    const id = rc.createImageData(PROC_W, PROC_H);
    for (let p = 0; p < PROC_W * PROC_H; p++) {
      id.data[p*4] = id.data[p*4+1] = id.data[p*4+2] = y[p]; id.data[p*4+3] = 255;
    }
    rc.putImageData(id, 0, 0);
    const blob = await oc.convertToBlob({type: 'image/jpeg', quality: 0.65});
    jpegBufs.push({tMs, buf: await blob.arrayBuffer()});
    if (i % 30 === 0) el('status').textContent = `encoding ${i}/${recFrames.length}…`;
  }

  const headerBytes = new TextEncoder().encode(JSON.stringify({
    version: 1, procW: PROC_W, procH: PROC_H, fx: FX, fy: FY, cx: CX, cy: CY,
    rciCalibrated: RciCalibrated, rci: Array.from(Rci)
  }));

  let total = 8 + 4 + headerBytes.length + 1;  // magic + hdrLen + hdr + EOF
  for (const {buf} of jpegBufs) total += 1 + 8 + 4 + buf.byteLength;
  for (const _ of recImu)       total += 1 + 8 + 24;

  const out = new ArrayBuffer(total);
  const dv = new DataView(out);
  let off = 0;

  // Magic "WSREC1\0\0"
  for (const b of [87,83,82,69,67,49,0,0]) dv.setUint8(off++, b);
  dv.setUint32(off, headerBytes.length, true); off += 4;
  new Uint8Array(out, off, headerBytes.length).set(headerBytes); off += headerBytes.length;

  // Interleave frame + IMU events sorted by timestamp
  const events = [
    ...jpegBufs.map(f => ({t: f.tMs, type: 1, buf: f.buf})),
    ...recImu.map(m => ({t: m.tMs, type: 2, ...m}))
  ].sort((a, b) => a.t - b.t);

  for (const ev of events) {
    dv.setUint8(off++, ev.type);
    dv.setFloat64(off, ev.t, true); off += 8;
    if (ev.type === 1) {
      const jb = new Uint8Array(ev.buf);
      dv.setUint32(off, jb.length, true); off += 4;
      new Uint8Array(out, off, jb.length).set(jb); off += jb.length;
    } else {
      for (const f of [ev.gx, ev.gy, ev.gz, ev.ax, ev.ay, ev.az])
        { dv.setFloat32(off, f, true); off += 4; }
    }
  }
  dv.setUint8(off, 0);  // EOF

  const url = URL.createObjectURL(new Blob([out], {type: 'application/octet-stream'}));
  Object.assign(document.createElement('a'), {href: url, download: `webslam-${Date.now()}.wsrec`}).click();
  URL.revokeObjectURL(url);
  el('status').textContent = `saved: ${jpegBufs.length} frames, ${recImu.length} IMU samples`;
  recFrames = []; recImu = [];
}

// ─── Replay: load .wsrec and feed through the engine at original speed ────────
async function loadAndReplay(file) {
  el('status').textContent = 'loading recording…';
  const buf = await file.arrayBuffer();
  const dv = new DataView(buf);
  let off = 0;

  if (dv.getUint8(0) !== 87 || dv.getUint8(1) !== 83) {
    el('status').textContent = 'not a valid .wsrec file'; return;
  }
  off += 8;

  const hdrLen = dv.getUint32(off, true); off += 4;
  const hdr = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, off, hdrLen)));
  off += hdrLen;
  if (hdr.rciCalibrated) { Rci = hdr.rci; RciCalibrated = true; }

  const events = [];
  while (off < buf.byteLength) {
    const type = dv.getUint8(off++);
    if (type === 0) break;
    const tMs = dv.getFloat64(off, true); off += 8;
    if (type === 1) {
      const len = dv.getUint32(off, true); off += 4;
      events.push({type: 1, tMs, data: buf.slice(off, off + len)});
      off += len;
    } else if (type === 2) {
      events.push({type: 2, tMs,
        gx: dv.getFloat32(off,    true), gy: dv.getFloat32(off+4,  true), gz: dv.getFloat32(off+8,  true),
        ax: dv.getFloat32(off+12, true), ay: dv.getFloat32(off+16, true), az: dv.getFloat32(off+20, true)});
      off += 24;
    }
  }

  // Decode all JPEG frames upfront
  const frameEvents = events.filter(e => e.type === 1);
  el('status').textContent = `decoding ${frameEvents.length} frames…`;
  for (const ev of frameEvents)
    ev.bitmap = await createImageBitmap(new Blob([ev.data], {type: 'image/jpeg'}));

  // Reset engine
  engine.enableMapping(); engine.resetScale(); scaleFed = 0; anchorWorld = null;

  const prevMode = mode;
  mode = 'replay';
  replayActive = true;
  el('recBtn').disabled = true;
  el('status').textContent = `replaying ${frameEvents.length} frames…`;

  const wallStart = performance.now(), recStart = events[0]?.tMs ?? 0;
  let lastImuTMs = 0;

  for (const ev of events) {
    if (!replayActive) break;
    // Pace to original speed
    const wait = (wallStart + ev.tMs - recStart) - performance.now();
    if (wait > 4) await new Promise(r => setTimeout(r, wait - 2));

    if (ev.type === 2) {
      gyroVec = [ev.gx, ev.gy, ev.gz];
      gyroMag = Math.hypot(ev.gx, ev.gy, ev.gz);
      if (RciCalibrated) {
        const dt = lastImuTMs > 0 ? (ev.tMs - lastImuTMs) / 1000 : 0.005;
        if (dt > 1e-4 && dt < 0.1) {
          const gcam = m3v(Rci, [ev.gx, ev.gy, ev.gz]);
          const acam = m3v(Rci, [ev.ax, ev.ay, ev.az]);
          engine.addImuSample(dt, gcam[0], gcam[1], gcam[2], acam[0], acam[1], acam[2]);
        }
      }
      lastImuTMs = ev.tMs;
    } else if (ev.type === 1) {
      pctx.drawImage(ev.bitmap, 0, 0);
      step();
    }
  }

  // Cleanup bitmaps
  for (const ev of frameEvents) { ev.bitmap?.close(); delete ev.bitmap; }

  replayActive = false;
  mode = prevMode;
  el('recBtn').disabled = false;
  el('status').textContent = 'replay done';
}

window.__anchor = () => anchorWorld;  // debug: read the anchored world point
window.__sphereUV = () => {           // debug: current sphere projection
  if (!anchorWorld) return null;
  const p = projectWorld(anchorWorld, engine.poseR(), engine.poseT());
  return { u: +p.u.toFixed(1), v: +p.v.toFixed(1), depth: +p.depth.toFixed(2), tracked: engine.mapTracked() };
};

main().catch((e) => { el('status').textContent = 'error: ' + e.message; });
