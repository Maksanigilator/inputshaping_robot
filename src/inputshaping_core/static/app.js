/* Tiny client for the inputshaping bench.
   Polls /api/status once per second, posts on button clicks. */

const q  = (sel) => document.querySelector(sel);
const qa = (sel) => Array.from(document.querySelectorAll(sel));

async function postJSON(url, body) {
  const r = await fetch(url, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body || {}),
  });
  const data = await r.json().catch(() => ({}));
  if (!r.ok || data.ok === false) {
    alert(`Error: ${data.error || r.statusText}`);
  }
  return data;
}

function fmt(n, d=3) { return Number.isFinite(n) ? n.toFixed(d) : '—'; }

async function refresh() {
  let s;
  try {
    s = await fetch('/api/status').then(r => r.json());
    q('#conn-state').textContent = 'connected';
    q('#conn-state').className = 'badge connected';
  } catch (e) {
    q('#conn-state').textContent = 'disconnected';
    q('#conn-state').className = 'badge error';
    return;
  }

  // Experiment.
  q('#exp-state').textContent  = s.experiment.state.toLowerCase();
  q('#exp-label').textContent  = s.experiment.profile_label || '—';
  q('#exp-lastv').textContent  = `${fmt(s.experiment.last_v)} m/s`;
  const pct = s.experiment.duration > 0
    ? Math.min(100, 100 * s.experiment.elapsed / s.experiment.duration) : 0;
  q('#exp-progress').style.width = `${pct}%`;
  q('#exp-elapsed').textContent =
    `${fmt(s.experiment.elapsed, 1)} s / ${fmt(s.experiment.duration, 1)} s`;

  // Shaper.
  const sh = s.shaper;
  if (sh.enabled) {
    q('#shaper-active').textContent =
      `${sh.name} @ ${fmt(sh.frequency, 1)} Hz (\u03b6=${fmt(sh.damping_ratio, 3)})`;
  } else {
    q('#shaper-active').textContent = 'disabled (pass-through)';
  }
  q('#shaper-delay').textContent = `${fmt(1000 * sh.delay, 1)} ms`;

  // IMU.
  q('#imu-rate').textContent =
    s.imu.sample_rate > 0 ? `${fmt(s.imu.sample_rate, 0)} Hz` : 'no data';
  q('#imu-dropped').textContent = `${s.imu.dropped_bytes} B / ${s.imu.dropped_frames} fr`;

  // Results list.
  const list = q('#results-list');
  if (!s.results || s.results.length === 0) {
    list.innerHTML = '<li class="muted">No runs yet.</li>';
  } else {
    list.innerHTML = '';
    for (const r of s.results) {
      const li = document.createElement('li');
      let recommendation = '';
      if (r.recommended) {
        recommendation = ` &mdash; recommended <b>${r.recommended.name}</b>
          @ ${fmt(r.recommended.frequency, 1)} Hz (V=${fmt(r.recommended.vibration, 2)})`;
      }
      li.innerHTML = `
        <div><b>${r.label}</b> &mdash; ${r.timestamp}${recommendation}</div>
        <div>${r.files.map(f => `<a href="/data/${f}" target="_blank">${f}</a>`).join(' ')}</div>
      `;
      list.appendChild(li);
    }
  }
}

function showPsdFields(method) {
  qa('.psd-only').forEach(el => { el.hidden = !el.classList.contains(`psd-${method}`); });
}

// Surface the server's clamp decision and the actual trapezoid the runner
// is about to publish, so users notice when their input was silently capped
// (e.g. a v_max=50 typo getting chopped to the 0.9 m/s hardware ceiling).
function showMotionInfo(exp) {
  const el = q('#motion-info');
  if (!el || !exp) return;
  const req = exp.requested || {};
  const app = exp.applied || {};
  const warn = (exp.warnings || []).join(' ');
  const dur = Number.isFinite(exp.duration) ? exp.duration.toFixed(2) : '—';
  el.classList.toggle('warn', !!warn);
  el.hidden = false;
  el.innerHTML = `
    <div><b>${exp.label || 'motion'}</b> &mdash; duration ${dur} s</div>
    <div class="muted">
      v_max: ${fmt(req.v_max)} &rarr; <b>${fmt(app.v_max)}</b> m/s,
      a_max: ${fmt(req.a_max)} &rarr; <b>${fmt(app.a_max)}</b> m/s<sup>2</sup>
    </div>
    ${warn ? `<div class="warn-text">⚠ ${warn}</div>` : ''}
  `;
}

document.addEventListener('DOMContentLoaded', () => {
  q('#shaper-apply').onclick = async () => {
    await postJSON('/api/shaper', {
      name: q('#shaper-name').value || null,
      frequency: parseFloat(q('#shaper-freq').value),
      damping_ratio: parseFloat(q('#shaper-damp').value),
      enabled: q('#shaper-enabled').checked && !!q('#shaper-name').value,
    });
    refresh();
  };

  const startMotion = async (sign) => {
    const resp = await postJSON('/api/motion', {
      distance: sign * parseFloat(q('#motion-dist').value),
      v_max: parseFloat(q('#motion-vmax').value),
      a_max: parseFloat(q('#motion-amax').value),
    });
    showMotionInfo(resp && resp.experiment);
    return resp;
  };
  q('#motion-fwd').onclick = () => startMotion(+1).then(refresh);
  q('#motion-bwd').onclick = () => startMotion(-1).then(refresh);
  q('#motion-cancel').onclick = () => postJSON('/api/cancel').then(refresh);

  q('#psd-method').onchange = (e) => showPsdFields(e.target.value);
  showPsdFields(q('#psd-method').value);

  q('#psd-go').onclick = async () => {
    const method = q('#psd-method').value;
    const direction = parseInt(q('#psd-dir').value, 10);
    const params = { direction, v_cruise: parseFloat(q('#psd-vcruise').value) };
    if (method === 'moving_chirp') {
      Object.assign(params, {
        max_distance: parseFloat(q('#psd-distance').value),
        n_passes: parseInt(q('#psd-passes').value, 10),
        f_start: parseFloat(q('#psd-fstart').value),
        f_end: parseFloat(q('#psd-fend').value),
        a_chirp_peak: parseFloat(q('#psd-apeak').value),
      });
    } else if (method === 'moving_impulse') {
      Object.assign(params, {
        pulse_width: parseFloat(q('#psd-pulse-w').value),
        pulse_dv: parseFloat(q('#psd-pulse-dv').value),
      });
    } else if (method === 'moving_step') {
      Object.assign(params, { step_dv: parseFloat(q('#psd-step-dv').value) });
    }
    await postJSON('/api/psd', { method, params });
    refresh();
  };

  setInterval(refresh, 1000);
  refresh();
  setInterval(refreshImu, 200);
  refreshImu();
});


// ---------------------------------------------------------------------------
// Live accelerometer plot
// ---------------------------------------------------------------------------

const IMU_COLORS = { ax: '#e25c5c', ay: '#67c98b', az: '#4fa3ff' };

// Smoothed Y range so the plot doesn't jitter when noise changes.
let imuYMin = -2, imuYMax = 12;

function fmtVal(v) { return Number.isFinite(v) ? v.toFixed(2) : '—'; }

async function refreshImu() {
  const canvas = q('#imu-canvas');
  if (!canvas) return;
  const win = parseFloat(q('#imu-window').value) || 2.0;
  let data;
  try {
    data = await fetch(`/api/imu/recent?duration=${win}&max_points=240`)
      .then(r => r.json());
  } catch (e) {
    return;
  }
  drawImu(canvas, data, win);
  const n = data.t ? data.t.length : 0;
  if (n > 0) {
    q('#imu-ax-val').textContent = fmtVal(data.ax[n - 1]);
    q('#imu-ay-val').textContent = fmtVal(data.ay[n - 1]);
    q('#imu-az-val').textContent = fmtVal(data.az[n - 1]);
  } else {
    q('#imu-ax-val').textContent = '—';
    q('#imu-ay-val').textContent = '—';
    q('#imu-az-val').textContent = '—';
  }
  q('#imu-window-info').textContent = `${n} samples / ${win.toFixed(1)} s`;
}

function drawImu(canvas, data, win) {
  // Match the internal canvas resolution to its rendered size so lines stay
  // crisp on hi-DPI screens and over an SSH-tunneled browser at any zoom.
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  if (canvas.width !== rect.width * dpr || canvas.height !== 260 * dpr) {
    canvas.width  = rect.width * dpr;
    canvas.height = 260 * dpr;
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const W = rect.width, H = 260;
  const pad = { l: 50, r: 14, t: 10, b: 22 };
  const pw = W - pad.l - pad.r;
  const ph = H - pad.t - pad.b;

  ctx.fillStyle = '#14161a';
  ctx.fillRect(0, 0, W, H);

  if (!data.t || data.t.length === 0) {
    ctx.fillStyle = '#8d949e';
    ctx.font = '12px system-ui';
    ctx.fillText('Waiting for IMU samples on /imu/data_raw …', pad.l, pad.t + ph / 2);
    return;
  }

  // Robust Y limits: percentile clip so a single spike doesn't squish the rest.
  const all = [].concat(data.ax, data.ay, data.az).filter(Number.isFinite);
  if (all.length) {
    all.sort((a, b) => a - b);
    const lo = all[Math.floor(all.length * 0.02)];
    const hi = all[Math.floor(all.length * 0.98)];
    const margin = Math.max(1, (hi - lo) * 0.15);
    // Smooth toward the new range to avoid axis flicker each frame.
    const newMin = lo - margin, newMax = hi + margin;
    imuYMin += 0.3 * (newMin - imuYMin);
    imuYMax += 0.3 * (newMax - imuYMax);
  }

  const tMin = -win, tMax = 0;
  const x2px = t => pad.l + (t - tMin) / (tMax - tMin) * pw;
  const y2px = y => pad.t + ph - (y - imuYMin) / (imuYMax - imuYMin) * ph;

  // Grid + Y labels (~5 horizontal lines).
  ctx.strokeStyle = '#252830';
  ctx.fillStyle = '#8d949e';
  ctx.font = '11px system-ui';
  ctx.textAlign = 'right';
  ctx.lineWidth = 1;
  const yStep = niceStep((imuYMax - imuYMin) / 5);
  const yFirst = Math.ceil(imuYMin / yStep) * yStep;
  for (let y = yFirst; y <= imuYMax; y += yStep) {
    const py = y2px(y);
    ctx.beginPath(); ctx.moveTo(pad.l, py); ctx.lineTo(pad.l + pw, py); ctx.stroke();
    ctx.fillText(y.toFixed(yStep < 1 ? 1 : 0), pad.l - 4, py + 3);
  }
  // Zero line, stronger.
  if (imuYMin < 0 && imuYMax > 0) {
    ctx.strokeStyle = '#3a3f47';
    const py = y2px(0);
    ctx.beginPath(); ctx.moveTo(pad.l, py); ctx.lineTo(pad.l + pw, py); ctx.stroke();
  }

  // X axis labels (time markers every ~0.5 s).
  ctx.textAlign = 'center';
  ctx.fillStyle = '#8d949e';
  const tStep = niceStep(win / 4);
  for (let t = -Math.floor(win / tStep) * tStep; t <= 0; t += tStep) {
    const px = x2px(t);
    ctx.strokeStyle = '#252830';
    ctx.beginPath(); ctx.moveTo(px, pad.t); ctx.lineTo(px, pad.t + ph); ctx.stroke();
    ctx.fillText(`${t.toFixed(1)}s`, px, pad.t + ph + 14);
  }

  // Plot each channel.
  for (const key of ['az', 'ay', 'ax']) {  // ax last so it lies on top
    const color = IMU_COLORS[key];
    ctx.strokeStyle = color;
    ctx.lineWidth = key === 'ax' ? 2.0 : 1.2;
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < data.t.length; i++) {
      const v = data[key][i];
      if (!Number.isFinite(v)) continue;
      const x = x2px(data.t[i]);
      const y = y2px(v);
      if (!started) { ctx.moveTo(x, y); started = true; }
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
}

function niceStep(raw) {
  if (!Number.isFinite(raw) || raw <= 0) return 1;
  const exp = Math.floor(Math.log10(raw));
  const base = Math.pow(10, exp);
  const norm = raw / base;
  let step;
  if (norm < 1.5) step = 1;
  else if (norm < 3) step = 2;
  else if (norm < 7) step = 5;
  else step = 10;
  return step * base;
}
