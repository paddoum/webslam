// Loads the actual compiled WASM module in Node and runs the same synthetic
// corner test as test_native.cpp, verifying the deployed artifact + Embind bridge.
import createSlamModule from '../web/wasm/slam.js';

const W = 320, H = 240;
const Module = await createSlamModule();
const engine = new Module.SlamEngine(W, H, 800);

// Build the synthetic RGBA frame: bright rectangle on dark background.
const rgba = new Uint8Array(W * H * 4);
const [rx0, ry0, rx1, ry1] = [100, 80, 220, 170];
for (let y = 0; y < H; y++) {
  for (let x = 0; x < W; x++) {
    const inside = x >= rx0 && x < rx1 && y >= ry0 && y < ry1;
    const v = inside ? 230 : 40;
    const i = (y * W + x) * 4;
    rgba[i] = v; rgba[i + 1] = v; rgba[i + 2] = v; rgba[i + 3] = 255;
  }
}

engine.inputView().set(rgba);
const n = engine.processFrame(W, H, 28);
const kp = engine.keypoints();
console.log(`WASM detected ${n} keypoints`);

const corners = [[rx0, ry0], [rx1 - 1, ry0], [rx0, ry1 - 1], [rx1 - 1, ry1 - 1]];
let found = 0;
for (const [cx, cy] of corners) {
  let best = Infinity;
  for (let i = 0; i < kp.length; i += 3) {
    best = Math.min(best, Math.hypot(kp[i] - cx, kp[i + 1] - cy));
  }
  const ok = best <= 4;
  console.log(`  corner (${cx},${cy}): nearest ${best.toFixed(1)} px ${ok ? 'OK' : 'MISS'}`);
  if (ok) found++;
}
console.log(found === 4 && n > 0 ? 'PASS (WASM)' : 'FAIL (WASM)');
process.exit(found === 4 ? 0 : 1);
