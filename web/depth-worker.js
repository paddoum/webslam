// M9.1 — monocular depth in a Web Worker (off the SLAM loop).
// Uses Transformers.js (Hugging Face) to run Depth-Anything-V2-small (quantized).
// Model is downloaded from the HF CDN on first use and cached by the browser.
import { pipeline, RawImage, env } from 'https://cdn.jsdelivr.net/npm/@huggingface/transformers@3.5.2';

env.allowLocalModels = false;  // fetch the model from the HF hub

let depth = null;

(async () => {
  try {
    // q8 keeps the download small; device:'wasm' avoids iOS WebGPU stalls.
    // progress_callback reports download/init so the UI isn't a static spinner.
    depth = await pipeline('depth-estimation', 'onnx-community/depth-anything-v2-small', {
      dtype: 'q8',
      device: 'wasm',
      progress_callback: (p) => postMessage({ type: 'progress', data: p }),
    });
    postMessage({ type: 'ready' });
  } catch (e) {
    postMessage({ type: 'error', message: 'load: ' + (e && e.message ? e.message : String(e)) });
  }
})();

onmessage = async (e) => {
  const m = e.data;
  if (m.type !== 'frame' || !depth) return;
  try {
    const img = new RawImage(new Uint8ClampedArray(m.data), m.width, m.height, 4);
    const out = await depth(img);
    // out.depth is a RawImage resized back to the INPUT resolution (easy pixel
    // mapping), normalized 0-255, ~proportional to inverse depth (closer=higher).
    const di = out.depth;
    const arr = Uint8Array.from(di.data);      // copy so we can transfer it
    postMessage({ type: 'depth', width: di.width, height: di.height, channels: di.channels, data: arr.buffer }, [arr.buffer]);
  } catch (e) {
    postMessage({ type: 'error', message: 'infer: ' + (e && e.message ? e.message : String(e)) });
  }
};
