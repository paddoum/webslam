// M9.1/M9.4 — monocular depth in a Web Worker (off the SLAM loop).
// Two backends, chosen by the worker URL's ?model= param:
//   'zip' (default) — ZipDepth-base NPU (6.1M params, MIT), a 12 MB fp16 ONNX
//                     served locally, run via onnxruntime-web (WASM SIMD).
//                     ~10-30x faster than DA2 -> depth at ~10 Hz instead of 2 Hz.
//   'da2'           — Depth-Anything-V2-small (q8, ~25 MB) via Transformers.js
//                     from the HF CDN. Kept as an A/B fallback (?depth=da2).
//
// Both post the same message: {type:'depth', width, height, channels:1, data,
// inferMs} where data is 0-255, higher = closer (relative inverse-ish depth) —
// the main thread's affine alignment (M9.2) is sign/scale-agnostic anyway.

const MODEL = new URLSearchParams(self.location.search).get('model') || 'zip';

const ZIP_W = 256, ZIP_H = 192;   // ZipDepth static input (exact 4:3, /32)

let inferFn = null;               // async (rgba, w, h) -> {data:Uint8Array, w, h, inferMs}

// ── ZipDepth via onnxruntime-web ──────────────────────────────────────────────
async function initZip() {
  const ort = await import('https://cdn.jsdelivr.net/npm/onnxruntime-web@1.20.1/dist/ort.wasm.min.mjs');
  ort.env.wasm.wasmPaths = 'https://cdn.jsdelivr.net/npm/onnxruntime-web@1.20.1/dist/';
  // No SharedArrayBuffer without cross-origin isolation -> single thread. SIMD is on by default.
  ort.env.wasm.numThreads = 1;

  postMessage({ type: 'progress', data: { status: 'downloading ZipDepth (12 MB)' } });
  const session = await ort.InferenceSession.create('./models/zipdepth_192x256_fp16.onnx', {
    executionProviders: ['wasm'],
  });

  const canvas = new OffscreenCanvas(ZIP_W, ZIP_H);
  const cctx = canvas.getContext('2d', { willReadFrequently: true });
  const input = new Float32Array(3 * ZIP_H * ZIP_W);
  const plane = ZIP_H * ZIP_W;

  return async (rgba, w, h) => {
    // Resize the RGBA frame to the model input with canvas bilinear.
    const bmp = await createImageBitmap(new ImageData(new Uint8ClampedArray(rgba), w, h));
    cctx.drawImage(bmp, 0, 0, ZIP_W, ZIP_H);
    bmp.close();
    const d = cctx.getImageData(0, 0, ZIP_W, ZIP_H).data;
    // RGBA u8 -> RGB float [0,1], NCHW (ZipDepth uses no mean/std normalization).
    for (let i = 0; i < plane; i++) {
      input[i]             = d[i * 4]     / 255;
      input[plane + i]     = d[i * 4 + 1] / 255;
      input[2 * plane + i] = d[i * 4 + 2] / 255;
    }
    const t0 = performance.now();
    const out = await session.run({ image: new ort.Tensor('float32', input, [1, 3, ZIP_H, ZIP_W]) });
    const inferMs = performance.now() - t0;
    const depth = out.depth.data;   // Float32Array [1,1,192,256]

    // Normalize to 0-255 and bilinear-upsample to the frame resolution so the
    // main thread's pixel mapping / densify path (expects PROC_WxPROC_H) works.
    let mn = Infinity, mx = -Infinity;
    for (let i = 0; i < depth.length; i++) { const v = depth[i]; if (v < mn) mn = v; if (v > mx) mx = v; }
    const scale = mx > mn ? 255 / (mx - mn) : 0;
    const outU8 = new Uint8Array(w * h);
    const sx = (ZIP_W - 1) / (w - 1), sy = (ZIP_H - 1) / (h - 1);
    for (let y = 0; y < h; y++) {
      const fy = y * sy, y0 = Math.min(ZIP_H - 2, fy | 0), ty = fy - y0;
      const r0 = y0 * ZIP_W, r1 = r0 + ZIP_W;
      for (let x = 0; x < w; x++) {
        const fx = x * sx, x0 = Math.min(ZIP_W - 2, fx | 0), tx = fx - x0;
        const v = (depth[r0 + x0] * (1 - tx) + depth[r0 + x0 + 1] * tx) * (1 - ty)
                + (depth[r1 + x0] * (1 - tx) + depth[r1 + x0 + 1] * tx) * ty;
        outU8[y * w + x] = ((v - mn) * scale + 0.5) | 0;
      }
    }
    return { data: outU8, w, h, inferMs };
  };
}

// ── Depth-Anything-V2-small via Transformers.js (fallback) ───────────────────
async function initDa2() {
  const { pipeline, RawImage, env } =
    await import('https://cdn.jsdelivr.net/npm/@huggingface/transformers@3.5.2');
  env.allowLocalModels = false;
  const depth = await pipeline('depth-estimation', 'onnx-community/depth-anything-v2-small', {
    dtype: 'q8',
    device: 'wasm',
    progress_callback: (p) => postMessage({ type: 'progress', data: p }),
  });
  return async (rgba, w, h) => {
    const img = new RawImage(new Uint8ClampedArray(rgba), w, h, 4);
    const t0 = performance.now();
    const out = await depth(img);
    const inferMs = performance.now() - t0;
    const di = out.depth;   // normalized 0-255 at input resolution
    return { data: Uint8Array.from(di.data), w: di.width, h: di.height, inferMs };
  };
}

(async () => {
  try {
    inferFn = MODEL === 'da2' ? await initDa2() : await initZip();
    postMessage({ type: 'ready', model: MODEL });
  } catch (e) {
    postMessage({ type: 'error', message: 'load: ' + (e && e.message ? e.message : String(e)) });
  }
})();

onmessage = async (e) => {
  const m = e.data;
  if (m.type !== 'frame' || !inferFn) return;
  try {
    const r = await inferFn(m.data, m.width, m.height);
    postMessage({ type: 'depth', width: r.w, height: r.h, channels: 1,
                  inferMs: r.inferMs, data: r.data.buffer }, [r.data.buffer]);
  } catch (e) {
    postMessage({ type: 'error', message: 'infer: ' + (e && e.message ? e.message : String(e)) });
  }
};
