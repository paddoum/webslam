// IMU diagnostics — measures whether the browser IMU is viable for tight VIO.
// Reports sample rate, timestamp jitter, at-rest bias/noise, gravity, and a
// camera↔IMU time-offset estimate (gyro vs frame-motion cross-correlation).
(() => {
  const el = (id) => document.getElementById(id);
  const D2R = Math.PI / 180;

  let running = false;
  const recent = [];          // recent {t, dt} for live rate/jitter
  let lastT = 0;
  let live = { gmag: 0, amag: 0, lat: 0 };
  let avail = { gyro: false, accel: false, accelG: false, interval: 0 };

  // collection buffers
  let stillBuf = null;        // {t,g,aLin,aG} while hold-still test active
  let syncGyro = null;        // {t, gmag} while sync test active

  function vmag(v) { return Math.hypot(v[0], v[1], v[2]); }

  function onMotion(e) {
    const now = performance.now();
    const rr = e.rotationRate, a = e.acceleration, aG = e.accelerationIncludingGravity;
    avail.gyro = !!rr; avail.accel = !!(a && a.x != null); avail.accelG = !!(aG && aG.x != null);
    if (e.interval) avail.interval = e.interval * 1000; // s→ms

    const g = rr ? [(rr.beta||0)*D2R, (rr.gamma||0)*D2R, (rr.alpha||0)*D2R] : [0,0,0];
    const aLin = a && a.x != null ? [a.x||0, a.y||0, a.z||0] : null;
    const aGv = aG && aG.x != null ? [aG.x||0, aG.y||0, aG.z||0] : null;
    live.gmag = vmag(g);
    live.amag = aGv ? vmag(aGv) : (aLin ? vmag(aLin) : 0);
    // event.timeStamp is ms since timeOrigin (comparable to performance.now()).
    live.lat = (e.timeStamp && e.timeStamp > 0) ? (now - e.timeStamp) : NaN;

    if (lastT) { recent.push({ t: now, dt: now - lastT }); if (recent.length > 120) recent.shift(); }
    lastT = now;

    if (stillBuf) stillBuf.push({ g, aLin, aG: aGv });
    if (syncGyro) syncGyro.push({ t: now, m: live.gmag });
  }

  function refreshLive() {
    el('hasGyro').textContent = avail.gyro ? 'yes' : 'NO';
    el('hasAccel').textContent = avail.accel ? 'yes' : 'no';
    el('hasAccelG').textContent = avail.accelG ? 'yes' : 'no';
    el('interval').textContent = avail.interval ? avail.interval.toFixed(1) + ' ms' : '—';
    // rate over the last second
    const now = performance.now();
    const win = recent.filter(s => now - s.t < 1000);
    el('rate').textContent = win.length ? win.length + ' Hz' : '—';
    if (recent.length > 8) {
      const dts = recent.map(s => s.dt);
      const mean = dts.reduce((a, b) => a + b, 0) / dts.length;
      const sd = Math.sqrt(dts.reduce((a, b) => a + (b - mean) * (b - mean), 0) / dts.length);
      el('jitter').textContent = sd.toFixed(1) + ' ms';
    }
    el('gnow').textContent = live.gmag.toFixed(3) + ' rad/s';
    el('anow').textContent = live.amag.toFixed(2) + ' m/s²';
    el('lat').textContent = isNaN(live.lat) ? 'n/a' : live.lat.toFixed(1) + ' ms';
    if (running) requestAnimationFrame(refreshLive);
  }

  el('startBtn').addEventListener('click', async () => {
    try {
      if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
        const res = await DeviceMotionEvent.requestPermission();
        if (res !== 'granted') { el('status').textContent = 'motion permission denied'; return; }
      }
      window.addEventListener('devicemotion', onMotion);
      running = true;
      el('startBtn').disabled = true; el('stillBtn').disabled = false; el('syncBtn').disabled = false;
      el('status').textContent = 'sensors live — run the hold-still and sync tests';
      requestAnimationFrame(refreshLive);
    } catch (err) { el('status').textContent = 'motion error: ' + err.name; }
  });

  // --- Hold-still: bias + noise ---
  el('stillBtn').addEventListener('click', () => {
    stillBuf = [];
    el('status').textContent = 'HOLD STILL on a table… (4s)';
    el('stillBtn').disabled = true;
    setTimeout(() => {
      const buf = stillBuf; stillBuf = null;
      el('stillBtn').disabled = false;
      if (!buf || buf.length < 10) { el('status').textContent = 'too few samples — try again'; return; }
      // gyro bias = |mean(g)|; noise = RMS of per-axis std
      const meanG = [0,0,0]; for (const s of buf) for (let k=0;k<3;k++) meanG[k]+=s.g[k];
      for (let k=0;k<3;k++) meanG[k]/=buf.length;
      let varG = 0; for (const s of buf) for (let k=0;k<3;k++) varG += (s.g[k]-meanG[k])**2;
      const gNoise = Math.sqrt(varG / (buf.length*3));
      el('gbias').textContent = vmag(meanG).toFixed(4) + ' rad/s';
      el('gnoise').textContent = gNoise.toFixed(4) + ' rad/s';
      // accel: mean magnitude (incl gravity) should be ~9.81; noise from linear accel
      const withG = buf.filter(s => s.aG);
      if (withG.length) {
        const m = withG.reduce((a,s)=>a+vmag(s.aG),0)/withG.length;
        el('amean').textContent = m.toFixed(2) + ' m/s² (g≈9.81)';
      }
      const withLin = buf.filter(s => s.aLin);
      if (withLin.length) {
        const meanA=[0,0,0]; for (const s of withLin) for(let k=0;k<3;k++) meanA[k]+=s.aLin[k];
        for(let k=0;k<3;k++) meanA[k]/=withLin.length;
        let varA=0; for (const s of withLin) for(let k=0;k<3;k++) varA+=(s.aLin[k]-meanA[k])**2;
        el('anoise').textContent = Math.sqrt(varA/(withLin.length*3)).toFixed(3) + ' m/s²';
      }
      el('status').textContent = 'hold-still done';
      computeVerdict(gNoise);
    }, 4000);
  });

  // --- Sync: cross-correlate gyro magnitude vs camera frame-motion energy ---
  el('syncBtn').addEventListener('click', async () => {
    el('syncBtn').disabled = true;
    let stream;
    try {
      stream = await navigator.mediaDevices.getUserMedia({ video: { facingMode: 'environment' }, audio: false });
    } catch (e) { el('status').textContent = 'camera needed for sync test: ' + e.name; el('syncBtn').disabled=false; return; }
    const video = el('video'); video.srcObject = stream; await video.play();
    const cv = el('cam'), ctx = cv.getContext('2d', { willReadFrequently: true });
    el('status').textContent = 'SHAKE / rotate the phone for 6s…';
    syncGyro = [];
    const camMotion = []; let prev = null; let stop = false;
    const loop = () => {
      ctx.drawImage(video, 0, 0, 160, 120);
      const d = ctx.getImageData(0, 0, 160, 120).data;
      let e = 0; if (prev) { for (let i = 0; i < d.length; i += 16) e += Math.abs(d[i] - prev[i]); }
      prev = d.slice();
      camMotion.push({ t: performance.now(), m: e });
      if (!stop) requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
    setTimeout(() => {
      stop = true;
      stream.getTracks().forEach(t => t.stop());
      const gyro = syncGyro; syncGyro = null;
      const r = crossCorrelate(camMotion.slice(2), gyro);  // drop first cam frames (no prev)
      if (r) {
        el('offset').textContent = (r.offset >= 0 ? '+' : '') + r.offset.toFixed(0) + ' ms';
        el('corr').textContent = r.corr.toFixed(2) + (r.corr > 0.5 ? ' (clear)' : ' (weak — shake harder)');
      } else { el('offset').textContent = 'n/a'; el('corr').textContent = 'no signal'; }
      el('syncBtn').disabled = false;
      el('status').textContent = 'sync test done';
    }, 6000);
  });

  // Resample two {t,m} series to a uniform grid, normalize, cross-correlate over lags.
  function crossCorrelate(a, b) {
    if (a.length < 20 || b.length < 20) return null;
    const t0 = Math.max(a[0].t, b[0].t), t1 = Math.min(a[a.length-1].t, b[b.length-1].t);
    if (t1 - t0 < 1500) return null;
    const dt = 5, n = Math.floor((t1 - t0) / dt);
    const sample = (arr) => {
      const out = new Float64Array(n); let j = 0;
      for (let i = 0; i < n; i++) {
        const t = t0 + i * dt;
        while (j < arr.length - 1 && arr[j+1].t < t) j++;
        const p = arr[Math.min(j, arr.length-1)], q = arr[Math.min(j+1, arr.length-1)];
        const f = q.t > p.t ? (t - p.t) / (q.t - p.t) : 0;
        out[i] = p.m + (q.m - p.m) * Math.max(0, Math.min(1, f));
      }
      // zero-mean, unit-norm
      let mean = 0; for (const v of out) mean += v; mean /= n;
      let nrm = 0; for (let i = 0; i < n; i++) { out[i] -= mean; nrm += out[i]*out[i]; }
      nrm = Math.sqrt(nrm) || 1; for (let i = 0; i < n; i++) out[i] /= nrm;
      return out;
    };
    const ca = sample(a), cb = sample(b);
    const maxLag = Math.floor(400 / dt);
    let best = -2, bestLag = 0;
    for (let lag = -maxLag; lag <= maxLag; lag++) {
      let s = 0;
      for (let i = 0; i < n; i++) { const j = i + lag; if (j >= 0 && j < n) s += ca[i] * cb[j]; }
      if (s > best) { best = s; bestLag = lag; }
    }
    return { offset: bestLag * dt, corr: best };  // +offset: gyro lags camera by offset ms
  }

  function computeVerdict(gNoise) {
    const now = performance.now();
    const rate = recent.filter(s => now - s.t < 1000).length;
    const dts = recent.map(s => s.dt);
    const mean = dts.reduce((a,b)=>a+b,0)/Math.max(1,dts.length);
    const jit = Math.sqrt(dts.reduce((a,b)=>a+(b-mean)**2,0)/Math.max(1,dts.length));
    const checks = [];
    checks.push(rate >= 50 ? ['rate', 2] : rate >= 30 ? ['rate', 1] : ['rate', 0]);
    checks.push(jit < 5 ? ['jitter', 2] : jit < 15 ? ['jitter', 1] : ['jitter', 0]);
    checks.push(gNoise < 0.02 ? ['noise', 2] : gNoise < 0.05 ? ['noise', 1] : ['noise', 0]);
    const min = Math.min(...checks.map(c => c[1]));
    const v = el('verdict');
    if (min === 2) { v.textContent = '✓ IMU looks usable for tight VIO'; v.className = 'good'; }
    else if (min === 1) { v.textContent = '~ IMU marginal — VIO may help but expect limits'; v.className = 'warn'; }
    else { v.textContent = '✗ IMU poor (' + checks.filter(c=>c[1]===0).map(c=>c[0]).join(', ') + ') — tight VIO unlikely to help'; v.className = 'bad'; }
  }
})();
