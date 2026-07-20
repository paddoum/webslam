// XFeat learned-feature worker (M15 / XFeat reloc channel).
// Mirrors depth-worker.js: onnxruntime-web from CDN, model served locally.
// The ONNX graph is only the XFeat backbone (feats / kpt-logits / reliability);
// keypoint decoding (softmax-fold, NMS, top-K, bilinear descriptor sampling,
// L2-norm) is post-processing done here in JS — the standard XFeat deployment.
//
// In:  { type:'frame', data: ArrayBuffer(RGBA), width, height }
// Out: { type:'ready' } | { type:'progress', data } | { type:'error', message }
//      | { type:'feat', n, kp: Float32Array(2n), desc: Float32Array(64n), inferMs }
//        kp are pixel coords in the SENT frame's resolution; desc are unit-norm.

const IN_W = 320, IN_H = 256;      // static ONNX input (multiples of 32)
const GW = IN_W / 8, GH = IN_H / 8; // descriptor/logit grid: 40 x 32
const TOP_K = 320;                  // keypoints returned (<= engine kMaxXFeat=512)
const DET_THRESH = 0.05;            // keypoint-heatmap detection threshold
const NMS_R = 2;                    // 5x5 non-max suppression (radius 2)

let session = null, ort = null;
let sentW = IN_W, sentH = IN_H;
const canvas = new OffscreenCanvas(IN_W, IN_H);
const cctx = canvas.getContext('2d', { willReadFrequently: true });
const gray = new Float32Array(IN_W * IN_H);  // 0-255 grayscale, net normalizes

async function init() {
  ort = await import('https://cdn.jsdelivr.net/npm/onnxruntime-web@1.20.1/dist/ort.wasm.min.mjs');
  ort.env.wasm.wasmPaths = 'https://cdn.jsdelivr.net/npm/onnxruntime-web@1.20.1/dist/';
  ort.env.wasm.numThreads = 1;
  postMessage({ type: 'progress', data: { status: 'downloading XFeat (2.7 MB)' } });
  session = await ort.InferenceSession.create('./models/xfeat_320x256.onnx', {
    executionProviders: ['wasm'],
  });
  postMessage({ type: 'ready' });
}

// RGBA frame -> 0-255 grayscale at IN_W x IN_H (letter-free stretch; keypoints
// are scaled back to the sent resolution on output).
async function preprocess(rgba, w, h) {
  sentW = w; sentH = h;
  const bmp = await createImageBitmap(new ImageData(new Uint8ClampedArray(rgba), w, h));
  cctx.drawImage(bmp, 0, 0, IN_W, IN_H);
  bmp.close();
  const d = cctx.getImageData(0, 0, IN_W, IN_H).data;
  for (let i = 0; i < IN_W * IN_H; i++)
    gray[i] = 0.299 * d[i * 4] + 0.587 * d[i * 4 + 1] + 0.114 * d[i * 4 + 2];
  return gray;
}

// Softmax over the 65 keypoint-logit channels, drop the dustbin (65th), and
// fold the 8x8 sub-cell probabilities into a full-resolution IN_H x IN_W map.
function keypointHeatmap(kpts) {
  const C = 65, HW = GW * GH;
  const full = new Float32Array(IN_W * IN_H);
  for (let gy = 0; gy < GH; gy++) {
    for (let gx = 0; gx < GW; gx++) {
      const base = gy * GW + gx;
      let mx = -Infinity;
      for (let c = 0; c < C; c++) { const v = kpts[c * HW + base]; if (v > mx) mx = v; }
      let sum = 0; const e = new Float32Array(64);
      for (let c = 0; c < 64; c++) { e[c] = Math.exp(kpts[c * HW + base] - mx); sum += e[c]; }
      sum += Math.exp(kpts[64 * HW + base] - mx);  // dustbin in the denominator
      for (let s = 0; s < 64; s++) {
        const sy = (s >> 3) & 7, sx = s & 7;         // 8x8 sub-cell (row-major)
        full[(gy * 8 + sy) * IN_W + (gx * 8 + sx)] = e[s] / sum;
      }
    }
  }
  return full;
}

// Bilinear-sample a GH x GW grid map at full-res pixel (px,py).
function sampleGrid(grid, ch, chIdx, px, py) {
  const gx = (px / IN_W) * GW - 0.5, gy = (py / IN_H) * GH - 0.5;
  let x0 = Math.floor(gx), y0 = Math.floor(gy);
  const fx = gx - x0, fy = gy - y0;
  const cl = (v, m) => v < 0 ? 0 : (v > m ? m : v);
  const x0c = cl(x0, GW - 1), x1c = cl(x0 + 1, GW - 1), y0c = cl(y0, GH - 1), y1c = cl(y0 + 1, GH - 1);
  const off = chIdx * GW * GH;
  const a = grid[off + y0c * GW + x0c], b = grid[off + y0c * GW + x1c];
  const c = grid[off + y1c * GW + x0c], d = grid[off + y1c * GW + x1c];
  return a * (1 - fx) * (1 - fy) + b * fx * (1 - fy) + c * (1 - fx) * fy + d * fx * fy;
}

function decode(feats, kptLogits, reliab) {
  const kmap = keypointHeatmap(kptLogits);
  // NMS on the full-res keypoint map -> candidate (x,y,score).
  const cand = [];
  for (let y = NMS_R; y < IN_H - NMS_R; y++) {
    for (let x = NMS_R; x < IN_W - NMS_R; x++) {
      const v = kmap[y * IN_W + x];
      if (v <= DET_THRESH) continue;
      let isMax = true;
      for (let dy = -NMS_R; dy <= NMS_R && isMax; dy++)
        for (let dx = -NMS_R; dx <= NMS_R; dx++)
          if (kmap[(y + dy) * IN_W + (x + dx)] > v) { isMax = false; break; }
      if (!isMax) continue;
      // reliability (GHxGW) bilinearly upsampled, times the detection prob.
      const rel = sampleGrid(reliab, 1, 0, x, y);
      cand.push([x, y, v * rel]);
    }
  }
  cand.sort((p, q) => q[2] - p[2]);
  const n = Math.min(TOP_K, cand.length);
  const kp = new Float32Array(2 * n), desc = new Float32Array(64 * n);
  const sx = sentW / IN_W, sy = sentH / IN_H;
  for (let i = 0; i < n; i++) {
    const [x, y] = cand[i];
    kp[2 * i] = x * sx; kp[2 * i + 1] = y * sy;   // back to sent resolution
    // bilinear-sample the 64-D descriptor, then L2-normalize
    let nrm = 0;
    for (let c = 0; c < 64; c++) { const v = sampleGrid(feats, 64, c, x, y); desc[64 * i + c] = v; nrm += v * v; }
    nrm = Math.sqrt(nrm) || 1;
    for (let c = 0; c < 64; c++) desc[64 * i + c] /= nrm;
  }
  return { n, kp, desc };
}

self.onmessage = async (e) => {
  const m = e.data;
  if (m.type !== 'frame' || !session) return;
  try {
    const g = await preprocess(m.data, m.width, m.height);
    const t0 = performance.now();
    const out = await session.run({ image: new ort.Tensor('float32', g, [1, 1, IN_H, IN_W]) });
    const inferMs = performance.now() - t0;
    const { n, kp, desc } = decode(out.feats.data, out.kpts.data, out.heatmap.data);
    postMessage({ type: 'feat', n, kp, desc, inferMs }, [kp.buffer, desc.buffer]);
  } catch (err) {
    postMessage({ type: 'error', message: String(err && err.message || err) });
  }
};

init().catch((err) => postMessage({ type: 'error', message: String(err && err.message || err) }));
