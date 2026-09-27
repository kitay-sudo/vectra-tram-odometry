// ---- Сцена песочницы: город, линия, вагон, погода ----
// Только рисование. Состояние даёт экран песочницы (js/sandbox_ui.js) из имитатора и ядра:
// где вагон, истинная скорость, режим модели, решения по тележкам, зоны погоды и застройки.
// Вагон - с двумя тележками в 7,55 м друг от друга (шкворни, tf организаторов), как в листе
// жюри; антенны GNSS master и rover - там же, где в tf (−9,873 и +2,563 м от передней тележки).
const TramScene = (() => {
  const OK = '14,154,167', WARN = '232,155,12', WET = '90,169,214';
  const PAL = {
    light: { skyTop: '#f7f9fb', skyBot: '#e3e8ed', g1: '#e6eaee', g2: '#dce1e6', far: '#d6dde4', mid: '#c4cdd6', roof: '#b7c1cb', win: 'rgba(80,96,112,0.28)', lit: 'rgba(80,96,112,0.28)',
      fog: '244,246,248', pole: 'rgba(40,48,58,0.5)', wire: 'rgba(40,48,58,0.42)', rail: '#4a525c', ballast: 'rgba(90,100,110,0.16)',
      sleeper: 'rgba(70,78,88,0.32)', body: '#fbfcfd', bodyStroke: 'rgba(0,0,0,0.2)', glass: 'rgba(24,30,40,0.88)', door: 'rgba(24,30,40,0.7)',
      roofBox: '#e4e8ec', bogie: '#2b3138', hub: '#8a939d', label: 'rgba(20,25,30,0.55)', tunnel: '#cdd3d9', earth: '#d6dbe0',
      tint: 'rgba(20,26,34,0.09)', lamp: 'rgba(232,155,12,0.8)', shadow: 'rgba(0,0,0,0.13)', urban: '#b9c3cd', urbanEdge: '#a7b2bd', road: '#d3d9df', lane: 'rgba(255,255,255,0.9)', tree: '#b3bec7', trunk: '#9aa6b0', verge0: 'rgba(170,185,170,0.25)', verge1: 'rgba(150,165,150,0.35)', curb: '#c3cad1', fence: 'rgba(60,70,80,0.35)', plat: '#cfd6dd', platEdge: '#e7c65a' },
    dark: { skyTop: '#0a0d11', skyBot: '#161b22', g1: '#12161c', g2: '#0d1015', far: '#252d39', mid: '#181e27', roof: '#202731', win: 'rgba(255,255,255,0.035)', lit: '',
      fog: '13,16,20', pole: 'rgba(200,210,220,0.32)', wire: 'rgba(200,210,220,0.28)', rail: '#8b95a1', ballast: 'rgba(150,160,170,0.09)',
      sleeper: 'rgba(150,160,170,0.22)', body: '#2a3038', bodyStroke: 'rgba(255,255,255,0.14)', glass: 'rgba(255,212,140,0.62)', door: 'rgba(255,212,140,0.45)',
      roofBox: '#343b44', bogie: '#06080a', hub: '#58616b', label: 'rgba(230,235,240,0.5)', tunnel: '#262c33', earth: '#1b2027',
      tint: 'rgba(0,0,0,0.28)', lamp: 'rgba(232,155,12,0.9)', shadow: 'rgba(0,0,0,0.45)', urban: '#1c232c', urbanEdge: '#27303b', road: '#141a21', lane: 'rgba(255,255,255,0.1)', tree: '#10151a', trunk: '#0c1014', verge0: 'rgba(20,28,24,0.6)', verge1: 'rgba(14,20,17,0.85)', curb: '#1b2128', fence: 'rgba(200,210,220,0.18)', plat: '#2a313a', platEdge: '#a88a2e' }
  };
  const hash = n => { const x = Math.sin(n * 127.1 + 311.7) * 43758.5453; return x - Math.floor(x); };
  // вагон: 15,3 м (как КТМ-19 с двумя тележками), шкворни в 7,55 м; начало координат - середина кузова
  const TL = 15.3, HALF = 7.55 / 2, DOORS = [-5.4, 0, 5.4];
  const ANT_MASTER = HALF - 9.873, ANT_ROVER = HALF + 2.563;
  // равномерное созвездие: не сбивается в кучу и не оставляет небо пустым
  const SATS = [0.05, 0.12, 0.03, 0.09, 0.06].map((hh, i, arr) => ({ o: i / arr.length, sp: 1 / 45, h: hh }));
  const RAIL_TINT = { rain: ['72,128,182', 0.85], ice: ['222,240,252', 0.95], leaves: ['150,98,40', 0.9] };
  const FONT = 'system-ui, "Segoe UI", Roboto, sans-serif';

  function create(cv) {
    const ctx = cv.getContext('2d');
    let W = 0, H = 0, gnssA = 1, wxA = 0, wxKind = 'rain', lastWx = 0;
    const SPARKS = [], SKID = [];
    const FLAKES = Array.from({ length: 220 }, (_, i) => ({ x: hash(i * 1.1), y: hash(i * 2.3), z: 0.4 + hash(i * 3.7) * 0.6, ph: hash(i * 4.9) * 6.28 }));
    function resize() {
      const dpr = Math.min(2, devicePixelRatio || 1);
      W = innerWidth; H = innerHeight;
      cv.width = W * dpr; cv.height = H * dpr;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }

    // S: { s (середина кузова, м линии), v, zAt, gradeAt, notch, mode, gnssOff, night, stops, iz, uz, wx, wheelAng, bog, fade, paused, dt }
    function draw(time, S) {
      const night = S.night, P = night ? PAL.dark : PAL.light, s = S.s, v = S.v;
      const mobile = W < 768;
      const ppm = Math.max(9, Math.min(22, W / 70));
      const railY = mobile ? H * 0.82 : H * 0.84;
      const horizonY = mobile ? H * 0.7 : H * 0.68;
      const camX = W * 0.5;
      const X = m => camX + (m - s) * ppm;
      const mMin = s - camX / ppm - 40, mMax = s + (W - camX) / ppm + 40;
      const wireY = railY - 5.6 * ppm;
      const dtR = Math.min(0.05, S.dt || 0);
      gnssA += ((S.gnssOff ? 0 : 1) - gnssA) * Math.min(1, dtR * 4);
      if (S.wx) wxKind = S.wx;
      wxA += ((S.wx ? 1 : 0) - wxA) * Math.min(1, dtR * 1.5);

      // sky and ground
      let g = ctx.createLinearGradient(0, 0, 0, horizonY);
      g.addColorStop(0, P.skyTop); g.addColorStop(1, P.skyBot);
      ctx.fillStyle = g; ctx.fillRect(0, 0, W, horizonY);
      g = ctx.createLinearGradient(0, horizonY, 0, railY);
      g.addColorStop(0, P.skyBot); g.addColorStop(1, P.g2);
      ctx.fillStyle = g; ctx.fillRect(0, horizonY, W, H - horizonY);

      // skylines
      const skyline = (par, stepPx, maxH, col, seed, detail) => {
        const off = s * ppm * par;
        const k0 = Math.floor(off / stepPx) - 1, k1 = Math.floor((off + W) / stepPx) + 1;
        for (let k = k0; k <= k1; k++) {
          const w = stepPx * (0.7 + hash(k * 3 + seed) * 0.45);
          const bh = maxH * (0.3 + hash(k * 7 + seed * 13) * 0.7);
          const x = k * stepPx - off + hash(k * 11 + seed) * stepPx * 0.2, y = horizonY - bh;
          ctx.fillStyle = col; ctx.fillRect(x, y, w, bh + 1);
          const r = hash(k * 17 + seed);
          ctx.fillStyle = detail ? P.roof : col;
          if (r < 0.35) ctx.fillRect(x + w * 0.2, y - 5, w * 0.25, 5);
          else if (r < 0.55) ctx.fillRect(x + w * 0.6, y - 9, 2, 9);
          if (!detail && !night) continue;
          const cw = detail ? 7 : 6, chh = detail ? 9 : 8, ww = detail ? 3 : 2, wh = detail ? 4 : 3;
          const cols = Math.floor((w - 8) / cw), rows = Math.floor((bh - 10) / chh);
          for (let i = 0; i < cols; i++) for (let j = 0; j < rows; j++) {
            const q = hash(k * 131 + i * 17 + j * 7 + seed);
            if (night) {
              if (q > (detail ? 0.74 : 0.86)) {
                const t = hash(q * 97.3), al = (detail ? 0.28 : 0.2) + hash(q * 41.1) * 0.3;
                const rgb = t < 0.7 ? '240,200,140' : t < 0.9 ? '250,220,175' : '185,205,235';
                ctx.fillStyle = `rgba(${rgb},${al})`; ctx.fillRect(x + 5 + i * cw, y + 7 + j * chh, ww, wh);
              }
            } else if (q > 0.15) { ctx.fillStyle = P.win; ctx.fillRect(x + 5 + i * cw, y + 7 + j * chh, ww, wh); }
          }
        }
      };
      if (night) {
        for (let i = 0; i < 90; i++) {
          const sx = hash(i * 1.37) * W, sy = hash(i * 7.91) * horizonY * 0.85;
          const tw = 0.35 + 0.45 * Math.abs(Math.sin(time * 0.0012 + i));
          ctx.fillStyle = `rgba(230,236,245,${tw * (0.4 + hash(i * 3.3) * 0.6)})`;
          ctx.fillRect(sx, sy, hash(i * 5.5) > 0.85 ? 2 : 1.2, hash(i * 5.5) > 0.85 ? 2 : 1.2);
        }
      }
      // moon at night, sun by day: in the free patch of sky between the chart and the right column
      if (mobile || W >= 1200) {
        const cx = mobile ? W * 0.82 : Math.min(W * 0.7, W - 420), cy = mobile ? H * 0.64 : H * 0.14;
        const r = mobile ? 14 : Math.max(18, Math.min(28, W * 0.016));
        if (night) {
          const halo = ctx.createRadialGradient(cx, cy, r * 0.8, cx, cy, r * 5);
          halo.addColorStop(0, 'rgba(210,222,240,0.16)'); halo.addColorStop(0.4, 'rgba(210,222,240,0.05)'); halo.addColorStop(1, 'rgba(210,222,240,0)');
          ctx.fillStyle = halo; ctx.fillRect(cx - r * 5, cy - r * 5, r * 10, r * 10);
          const disc = ctx.createRadialGradient(cx - r * 0.3, cy - r * 0.3, r * 0.1, cx, cy, r);
          disc.addColorStop(0, '#f3f5f8'); disc.addColorStop(1, '#cfd6df');
          ctx.fillStyle = disc; ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.fill();
          ctx.fillStyle = 'rgba(150,162,178,0.35)';
          [[-0.3, -0.2, 0.22], [0.25, 0.1, 0.16], [-0.05, 0.4, 0.12], [0.35, -0.35, 0.09]].forEach(([dx, dy, rr]) => {
            ctx.beginPath(); ctx.arc(cx + dx * r, cy + dy * r, rr * r, 0, Math.PI * 2); ctx.fill();
          });
        } else {
          const halo = ctx.createRadialGradient(cx, cy, r * 0.6, cx, cy, r * 6);
          halo.addColorStop(0, 'rgba(255,214,140,0.45)'); halo.addColorStop(0.3, 'rgba(255,214,140,0.14)'); halo.addColorStop(1, 'rgba(255,214,140,0)');
          ctx.fillStyle = halo; ctx.fillRect(cx - r * 6, cy - r * 6, r * 12, r * 12);
          const disc = ctx.createRadialGradient(cx, cy, 0, cx, cy, r);
          disc.addColorStop(0, '#fff6dc'); disc.addColorStop(0.7, '#ffd98a'); disc.addColorStop(1, '#f7c45e');
          ctx.fillStyle = disc; ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.fill();
        }
      }
      skyline(0.1, 70, H * 0.2, P.far, 1, false);
      g = ctx.createLinearGradient(0, horizonY - H * 0.12, 0, horizonY + H * 0.04);
      g.addColorStop(0, `rgba(${P.fog},0)`); g.addColorStop(0.8, `rgba(${P.fog},0.55)`); g.addColorStop(1, `rgba(${P.fog},0)`);
      ctx.fillStyle = g; ctx.fillRect(0, horizonY - H * 0.12, W, H * 0.16);
      skyline(0.3, 96, H * 0.12, P.mid, 2, true);

      // ground depth: far street with lane marks
      {
        const gap = railY - horizonY;
        const ry = horizonY + gap * 0.16, rh = Math.max(4, gap * 0.07);
        ctx.fillStyle = P.road; ctx.fillRect(0, ry, W, rh);
        const offR = s * ppm * 0.45;
        ctx.fillStyle = P.lane;
        for (let x = -(offR % 46); x < W; x += 46) ctx.fillRect(x, ry + rh / 2 - 0.5, 20, 1);
      }
      const gapG = railY - horizonY, curbY = railY - gapG * 0.2;

      // dense urban zone: tall blocks close to the track
      const uzList = S.uz || [], izList = S.iz || [];
      const inList = (list, m) => list.some(z => m > z[0] && m < z[1]);
      const wetAt = m => { for (const z of izList) if (m > z[0] && m < z[1]) return true; return false; };
      for (const uz of uzList) {
        const ua = Math.max(uz[0] - 6, Math.floor((mMin - 40) / 16) * 16), ub = Math.min(uz[1] + 6, mMax + 40);
        if (X(ub) > -50 && X(ua) < W + 50) {
          for (let m = Math.floor(ua / 16) * 16; m < ub; m += 16) {
            if (m < ua - 8) continue;
            const k = Math.round(m / 16), bw = (11 + hash(k * 3.7) * 5) * ppm;
            const edge = Math.min(1, (m - (uz[0] - 6)) / 30, (uz[1] + 6 - m) / 30);
            const bh = (railY - H * 0.2) * (0.5 + 0.5 * hash(k * 5.3)) * Math.max(0.35, edge);
            const bx = X(m), by = curbY + 1 - bh;
            ctx.fillStyle = P.urban; ctx.fillRect(bx, by, bw, bh);
            ctx.fillStyle = P.urbanEdge; ctx.fillRect(bx, by, bw, Math.max(3, 0.25 * ppm));
            const cw = 1.6 * ppm, chh = 2.2 * ppm;
            for (let i = 0; i < Math.floor((bw - 0.8 * ppm) / cw); i++) for (let j = 0; j < Math.floor((bh - 1.5 * ppm) / chh); j++) {
              const q = hash(k * 91 + i * 13 + j * 7);
              if (night) { if (q > 0.72) { ctx.fillStyle = `rgba(240,200,140,${0.22 + hash(q * 7) * 0.3})`; ctx.fillRect(bx + 0.6 * ppm + i * cw, by + 1 * ppm + j * chh, 0.8 * ppm, 1.1 * ppm); } }
              else if (q > 0.1) { ctx.fillStyle = P.win; ctx.fillRect(bx + 0.6 * ppm + i * cw, by + 1 * ppm + j * chh, 0.8 * ppm, 1.1 * ppm); }
            }
          }
        }
      }

      // curb and low fence in front of the buildings
      ctx.fillStyle = P.curb; ctx.fillRect(0, curbY, W, Math.max(2, gapG * 0.025));
      const offC = s * ppm * 0.92;
      ctx.fillStyle = P.fence;
      for (let x = -(offC % 26); x < W; x += 26) ctx.fillRect(x, curbY - gapG * 0.06, 1.5, gapG * 0.06);
      ctx.fillRect(0, curbY - gapG * 0.05, W, 1);

      // ---- elevation: the camera follows the tram's height, the track ahead rises or dips ----
      const EX = 1.6, zS = S.zAt(s);
      const maxDown = Math.max(12, H - 62 - railY - 1.1 * ppm), maxUp = railY * 0.35;
      const YO = m => {
        const y = -(S.zAt(m) - zS) * ppm * EX;
        return y > 0 ? maxDown * Math.tanh(y / maxDown) : maxUp * Math.tanh(y / maxUp);
      };
      const RY = m => railY + YO(m);
      const m0 = Math.floor(mMin / 2) * 2;

      // catenary: contact wire follows the track
      ctx.strokeStyle = P.wire; ctx.lineWidth = 1;
      ctx.beginPath(); for (let m = m0; m <= mMax; m += 2) ctx.lineTo(X(m), wireY + YO(m)); ctx.stroke();
      const SP = 35;
      for (let m = Math.floor(mMin / SP) * SP; m <= mMax; m += SP) {
        const n = m + SP, yo = YO(m);
        const x = X(m);
        ctx.fillStyle = P.pole;
        ctx.fillRect(x - Math.max(1.5, 0.1 * ppm), railY + yo - 7.2 * ppm, Math.max(3, 0.2 * ppm), 7.5 * ppm);
        ctx.fillRect(x - Math.max(3, 0.25 * ppm), railY + yo - 0.1 * ppm, Math.max(6, 0.5 * ppm), 0.35 * ppm);
        if (night) {
          const lx = x - 0.9 * ppm, lyy = railY + yo - 7.2 * ppm;
          const LR = 5 * ppm;
          const lg = ctx.createRadialGradient(lx, lyy, 0, lx, lyy, LR);
          lg.addColorStop(0, 'rgba(255,214,150,0.26)'); lg.addColorStop(0.15, 'rgba(255,214,150,0.14)');
          lg.addColorStop(0.4, 'rgba(255,214,150,0.05)'); lg.addColorStop(0.7, 'rgba(255,214,150,0.012)'); lg.addColorStop(1, 'rgba(255,214,150,0)');
          ctx.fillStyle = lg; ctx.fillRect(lx - LR, lyy - LR, LR * 2, LR * 2);
          ctx.fillStyle = P.pole; ctx.fillRect(x - 1.2 * ppm, lyy - 0.12 * ppm, 1.2 * ppm, 0.12 * ppm + 1);
          ctx.fillStyle = '#ffe2ad'; ctx.fillRect(lx - 0.25 * ppm, lyy, 0.5 * ppm, Math.max(2, 0.1 * ppm));
        }
        const xa = X(m), xb = X(n), ya = railY + yo - 6.6 * ppm, yb = railY + YO(n) - 6.6 * ppm;
        ctx.strokeStyle = P.wire; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(xa, ya); ctx.quadraticCurveTo((xa + xb) / 2, (ya + yb) / 2 + 1.4 * ppm, xb, yb); ctx.stroke();
        for (let k = 1; k < 5; k++) {
          const t = k / 5, xd = xa + (xb - xa) * t;
          const yd = (1 - t) * (1 - t) * ya + 2 * (1 - t) * t * ((ya + yb) / 2 + 1.4 * ppm) + t * t * yb;
          ctx.beginPath(); ctx.moveTo(xd, yd); ctx.lineTo(xd, wireY + YO(m + SP * t)); ctx.stroke();
        }
      }

      // embankment so rises and dips sit on solid ground
      ctx.fillStyle = P.g2;
      ctx.beginPath(); ctx.moveTo(-10, H);
      for (let m = m0; m <= mMax; m += 2) ctx.lineTo(X(m), RY(m) + 0.5 * ppm);
      ctx.lineTo(W + 10, H); ctx.closePath(); ctx.fill();

      // platforms with waiting passengers (behind the track); p - передняя тележка вагона на остановке
      (S.stops || []).forEach((q, qi) => {
        const c = q.p - HALF, pa = X(c - 9), pb = X(c + 9);
        if (pb < -40 || pa > W + 40) return;
        const yb = RY(c);
        ctx.fillStyle = P.plat; ctx.fillRect(pa, yb - 0.55 * ppm, pb - pa, 0.55 * ppm);
        ctx.fillStyle = P.platEdge; ctx.fillRect(pa, yb - 0.55 * ppm, pb - pa, Math.max(2, 0.08 * ppm));
        const sa = X(c + 2.6), sb = X(c + 8), roofY = yb - 3.4 * ppm;
        ctx.fillStyle = P.pole;
        ctx.fillRect(sa, roofY, Math.max(2, 0.12 * ppm), 2.85 * ppm);
        ctx.fillRect(sb - Math.max(2, 0.12 * ppm), roofY, Math.max(2, 0.12 * ppm), 2.85 * ppm);
        ctx.fillStyle = P.roofBox; ctx.fillRect(sa - 0.4 * ppm, roofY - 0.2 * ppm, sb - sa + 0.8 * ppm, 0.3 * ppm);
        ctx.fillStyle = P.tint; ctx.fillRect(sa, roofY + 0.1 * ppm, sb - sa, 2.75 * ppm);
        // passengers: boarding ones walk to the nearest door and disappear
        const boardP = q.boardP || 0;
        for (let i = 0; i < q.pax; i++) {
          const hsh = hash(qi * 37 + i * 3.3);
          let px = c - 8 + hsh * 16;
          if (boardP > 0) {
            const door = DOORS.reduce((b, d) => Math.abs(c + d - px) < Math.abs(c + b - px) ? d : b, 0);
            const my = Math.min(1, Math.max(0, boardP * 1.6 - hash(i * 7.7) * 0.6));
            if (my >= 1) continue;
            px = px + (c + door - px) * my;
          }
          const x = X(px), hgt = (1.5 + hash(i * 5.1 + qi) * 0.3) * ppm, y = yb - 0.55 * ppm;
          const cols = night ? ['#39424e', '#4a4452', '#3d4a47', '#514a3f'] : ['#7d8793', '#8e7f8f', '#7a8f88', '#98876f'];
          ctx.fillStyle = cols[Math.floor(hash(i * 2.9 + qi) * 4)];
          ctx.fillRect(x - 0.18 * ppm, y - hgt * 0.78, 0.36 * ppm, hgt * 0.78);
          ctx.beginPath(); ctx.arc(x, y - hgt * 0.78 - 0.2 * ppm, 0.2 * ppm, 0, Math.PI * 2); ctx.fill();
        }
        const sx = X(c - 10);
        ctx.fillStyle = P.pole; ctx.fillRect(sx - 1, yb - 3.6 * ppm, 2, 3.05 * ppm);
        ctx.font = `500 ${Math.max(10, Math.min(13, 0.6 * ppm))}px ${FONT}`;
        const tw = ctx.measureText(q.n).width + 12;
        ctx.fillStyle = P.bogie; ctx.fillRect(sx - tw / 2, yb - 4.5 * ppm, tw, Math.max(16, 0.9 * ppm));
        ctx.fillStyle = '#f2f4f6'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.fillText(q.n, sx, yb - 4.5 * ppm + Math.max(16, 0.9 * ppm) / 2);
        ctx.textBaseline = 'alphabetic';
      });

      // track bed
      ctx.fillStyle = P.ballast;
      ctx.beginPath();
      for (let m = m0; m <= mMax; m += 2) ctx.lineTo(X(m), RY(m));
      for (let m = Math.floor(mMax / 2) * 2; m >= m0; m -= 2) ctx.lineTo(X(m), RY(m) + 0.55 * ppm);
      ctx.closePath(); ctx.fill();
      ctx.fillStyle = P.sleeper;
      for (let m = Math.floor(mMin / 0.75) * 0.75; m <= mMax; m += 0.75) ctx.fillRect(X(m), RY(m) + 1, Math.max(2, 0.25 * ppm), Math.max(2, 0.22 * ppm));
      ctx.strokeStyle = P.rail; ctx.lineWidth = Math.max(2, 0.14 * ppm); ctx.beginPath();
      for (let m = m0; m <= mMax; m += 1) ctx.lineTo(X(m), RY(m) - 0.07 * ppm);
      ctx.stroke();
      // wet, icy or leafy rails: only the rail itself changes colour, fading in and out at the ends
      for (const iz of izList) {
        const a = Math.max(Math.ceil(iz[0]), Math.floor(mMin)), b = Math.min(Math.floor(iz[1]), Math.ceil(mMax));
        if (b <= a) continue;
        const [rgb, alMax] = RAIL_TINT[iz[2]] || RAIL_TINT.rain;
        ctx.lineWidth = Math.max(2, 0.16 * ppm); ctx.lineCap = 'butt';
        for (let m = a; m < b; m++) {
          const al = Math.min(1, (m - iz[0]) / 20, (iz[1] - m - 1) / 20);
          if (al <= 0) continue;
          ctx.strokeStyle = `rgba(${rgb},${alMax * al})`;
          ctx.beginPath(); ctx.moveTo(X(m), RY(m) - 0.08 * ppm); ctx.lineTo(X(m + 1) + 0.5, RY(m + 1) - 0.08 * ppm); ctx.stroke();
        }
      }

      // data line along the rail
      const flow = time * 0.0025;
      ctx.lineWidth = Math.max(2, 0.1 * ppm); ctx.lineCap = 'round';
      for (let m = Math.floor(mMin / 2) * 2; m <= mMax; m += 2) {
        const mm = m + (flow % 2);
        const tun = inList(uzList, mm), wt = wetAt(mm);
        ctx.strokeStyle = `rgba(${tun ? WARN : wt ? WET : OK},0.75)`;
        ctx.beginPath(); ctx.moveTo(X(mm), RY(mm) + 0.8 * ppm); ctx.lineTo(X(mm + 1.1), RY(mm + 1.1) + 0.8 * ppm); ctx.stroke();
      }
      ctx.lineCap = 'butt';

      // satellites and links (to the master antenna)
      const top = railY - 0.62 * ppm - 2.75 * ppm;
      const ant = { x: X(s + ANT_MASTER), y: top - 0.35 * ppm };
      const tsec = time / 1000;
      const sats = SATS.map(q => {
        const u = (q.o + tsec * q.sp) % 1;
        const xMin = mobile ? -40 : W * 0.5;
        const x = xMin + u * (W + 40 - xMin);
        const yTop = mobile ? H * (0.36 + q.h * 0.6) : H * q.h;
        const sag = (horizonY - yTop) * (mobile ? 0.35 : 0.5);
        const y = yTop + sag * (2 * u - 1) ** 2;
        const vis = Math.min(1, Math.sin(Math.PI * u) * 3);
        return [x, y, vis];
      });
      ctx.setLineDash([2, 6]); ctx.lineDashOffset = -time * 0.03;
      sats.forEach(([sx, sy, vis]) => {
        if (gnssA * vis > 0.02) {
          ctx.strokeStyle = `rgba(${OK},${0.55 * gnssA * vis})`; ctx.lineWidth = 1.2;
          ctx.beginPath(); ctx.moveTo(ant.x, ant.y); ctx.lineTo(sx, sy); ctx.stroke();
        }
      });
      ctx.setLineDash([]);
      sats.forEach(([sx, sy, vis]) => {
        ctx.save(); ctx.translate(sx, sy); ctx.globalAlpha = (0.25 + 0.75 * gnssA) * vis;
        ctx.fillStyle = P.label; ctx.strokeStyle = P.label; ctx.lineWidth = 1.2;
        ctx.rotate(Math.PI / 4); ctx.fillRect(-3.5, -3.5, 7, 7); ctx.rotate(-Math.PI / 4);
        ctx.strokeRect(-15, -2.5, 8, 5); ctx.strokeRect(7, -2.5, 8, 5);
        if (gnssA < 0.5) {
          ctx.strokeStyle = `rgba(${WARN},${1 - gnssA * 2})`; ctx.lineWidth = 1.6;
          ctx.beginPath(); ctx.moveTo(-5, 10); ctx.lineTo(5, 20); ctx.moveTo(5, 10); ctx.lineTo(-5, 20); ctx.stroke();
        }
        ctx.restore();
      });

      const tilt = -Math.atan(Math.tan(S.gradeAt(s)) * 1.6);
      ctx.save();
      ctx.translate(X(s), railY); ctx.rotate(tilt); ctx.translate(-X(s), -railY);
      drawTram(P, ppm, railY, X, wireY, S, night);
      ctx.restore();
      drawBogieLabels(S, ppm, X, RY, tilt, railY, mobile, night);

      // weather particles
      if (wxA > 0.01) drawWeather(time, v, night);

      // readability fade behind the dashboard
      const fw = mobile ? W : Math.min(W, W * 0.62 + 120), fh = mobile ? H * 0.72 : H * 0.66;
      g = ctx.createLinearGradient(0, 0, fw, 0);
      g.addColorStop(0, `rgba(${P.fog},0.9)`); g.addColorStop(mobile ? 1 : 0.7, `rgba(${P.fog},0.8)`); g.addColorStop(1, `rgba(${P.fog},${mobile ? 0.8 : 0})`);
      ctx.fillStyle = g; ctx.fillRect(0, 0, fw, fh);
      for (let i = 0; i < 12; i++) { ctx.globalAlpha = 1 - (i + 1) / 13; ctx.fillRect(0, fh + i * H * 0.008, fw, H * 0.008); }
      ctx.globalAlpha = 1;
      if (S.fade > 0) { ctx.fillStyle = `rgba(${P.fog},${S.fade})`; ctx.fillRect(0, 0, W, H); }
    }

    function drawWeather(time, v, night) {
      const t = time / 1000, dt = Math.min(0.05, t - lastWx || 0); lastWx = t;
      const n = Math.round(FLAKES.length * wxA);
      if (wxKind === 'ice') {
        ctx.fillStyle = night ? 'rgba(230,236,245,0.85)' : 'rgba(255,255,255,0.95)';
        for (let i = 0; i < n; i++) {
          const f = FLAKES[i];
          f.y += dt * (0.05 + 0.08 * f.z); f.x += dt * (0.01 * Math.sin(t + f.ph) - v * 0.004 * f.z);
          if (f.y > 1) { f.y -= 1; f.x = Math.random(); } if (f.x < 0) f.x += 1; if (f.x > 1) f.x -= 1;
          const r = 1 + 2.2 * f.z;
          ctx.globalAlpha = 0.35 + 0.6 * f.z;
          ctx.beginPath(); ctx.arc(f.x * W, f.y * H, r, 0, Math.PI * 2); ctx.fill();
        }
      } else if (wxKind === 'leaves') {
        const cols = night ? ['#7a5a33', '#6b4a2a', '#84622f'] : ['#c98b3a', '#a4652a', '#d9a441', '#b5552e'];
        const nl = Math.round(n * 0.35);
        for (let i = 0; i < nl; i++) {
          const f = FLAKES[i];
          f.y += dt * (0.035 + 0.05 * f.z); f.x += dt * (0.03 * Math.sin(t * 1.3 + f.ph) - 0.02 - v * 0.004 * f.z);
          if (f.y > 1) { f.y -= 1; f.x = Math.random(); } if (f.x < 0) f.x += 1; if (f.x > 1) f.x -= 1;
          const sz = 2.5 + 4 * f.z, a = t * (1.5 + f.z) + f.ph;
          ctx.save(); ctx.translate(f.x * W, f.y * H); ctx.rotate(a); ctx.scale(1, 0.35 + 0.65 * Math.abs(Math.sin(a * 0.7)));
          ctx.globalAlpha = 0.55 + 0.4 * f.z; ctx.fillStyle = cols[i % cols.length];
          ctx.beginPath(); ctx.ellipse(0, 0, sz, sz * 0.55, 0, 0, Math.PI * 2); ctx.fill();
          ctx.restore();
        }
      } else {
        ctx.strokeStyle = night ? 'rgba(170,190,215,0.55)' : 'rgba(90,110,135,0.45)'; ctx.lineWidth = 1;
        ctx.beginPath();
        for (let i = 0; i < n; i++) {
          const f = FLAKES[i];
          f.y += dt * (0.9 + 0.8 * f.z); f.x -= dt * (0.08 + v * 0.004);
          if (f.y > 1) { f.y -= 1; f.x = Math.random(); } if (f.x < 0) f.x += 1;
          const x = f.x * W, y = f.y * H, L = 10 + 14 * f.z;
          ctx.moveTo(x, y); ctx.lineTo(x - L * 0.18, y + L);
        }
        ctx.globalAlpha = 1; ctx.stroke();
      }
      ctx.globalAlpha = 1;
    }

    function drawTram(P, ppm, railY, X, wireY, S, night) {
      const s = S.s, v = S.v;
      const left = X(s - TL / 2), right = X(s + TL / 2);
      const bottom = railY - 0.62 * ppm, top = bottom - 2.75 * ppm;
      const r = 0.5 * ppm, nose = 1.3 * ppm;

      // pantograph
      const bx = X(s + 0.4), by = top - 0.4 * ppm;
      ctx.strokeStyle = P.bogie; ctx.lineWidth = Math.max(1.5, 0.08 * ppm);
      const kneeX = bx + 1.1 * ppm, kneeY = by - (by - wireY) * 0.5;
      ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(kneeX, kneeY); ctx.lineTo(bx - 0.2 * ppm, wireY + 0.1 * ppm); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(bx - 1 * ppm, wireY + 0.1 * ppm); ctx.lineTo(bx + 0.7 * ppm, wireY + 0.1 * ppm); ctx.stroke();
      // pantograph sparks: at night, more often in rain or snow and under traction
      const sparkP = S.paused ? 0 : (night ? 1 : 0.25) * (S.wx ? 3 : 0.6) * (S.notch > 0 ? 2 : 0.6) * (v > 1 ? 1 : 0);
      if (Math.random() < 0.02 * sparkP) {
        const n = 3 + Math.floor(Math.random() * 6);
        for (let i = 0; i < n; i++) SPARKS.push({ x: bx - 0.2 * ppm + (Math.random() - 0.5) * 0.6 * ppm, y: wireY + 0.1 * ppm, vx: -(0.5 + Math.random() * 2.2) * ppm, vy: (Math.random() - 0.7) * 1.6 * ppm, life: 0.25 + Math.random() * 0.35, age: 0, flash: i === 0 });
      }
      const sdt = Math.min(0.05, S.dt || 0);
      ctx.save(); ctx.globalCompositeOperation = 'lighter';
      for (let i = SPARKS.length - 1; i >= 0; i--) {
        const p = SPARKS[i]; p.age += sdt;
        if (p.age > p.life) { SPARKS.splice(i, 1); continue; }
        p.vy += 4 * ppm * sdt; p.x += p.vx * sdt; p.y += p.vy * sdt;
        const k = 1 - p.age / p.life;
        if (p.flash && p.age < 0.08) {
          const fr = 1.4 * ppm, fg = ctx.createRadialGradient(p.x, p.y, 0, p.x, p.y, fr);
          fg.addColorStop(0, `rgba(190,215,255,${0.55 * (1 - p.age / 0.08)})`); fg.addColorStop(1, 'rgba(190,215,255,0)');
          ctx.fillStyle = fg; ctx.fillRect(p.x - fr, p.y - fr, fr * 2, fr * 2);
        }
        ctx.fillStyle = `rgba(215,230,255,${0.9 * k})`;
        ctx.fillRect(p.x, p.y, 1.6, 1.6);
      }
      ctx.restore();

      // roof boxes
      ctx.fillStyle = P.roofBox; ctx.strokeStyle = P.bodyStroke; ctx.lineWidth = 1;
      [[-2.0, 2.0], [-5.1, -3.4], [3.4, 5.1]].forEach(([a, b]) => {
        ctx.beginPath(); ctx.roundRect(X(s + a), top - 0.4 * ppm, (b - a) * ppm, 0.5 * ppm, 0.15 * ppm); ctx.fill(); ctx.stroke();
      });

      // body path
      const body = new Path2D();
      body.moveTo(left + nose, top);
      body.lineTo(right - nose, top);
      body.quadraticCurveTo(right, top, right, top + nose);
      body.lineTo(right, bottom - r);
      body.quadraticCurveTo(right, bottom, right - r, bottom);
      body.lineTo(left + r, bottom);
      body.quadraticCurveTo(left, bottom, left, bottom - r);
      body.lineTo(left, top + nose);
      body.quadraticCurveTo(left, top, left + nose, top);
      body.closePath();
      ctx.fillStyle = P.body; ctx.fill(body);

      ctx.save(); ctx.clip(body);
      // accent stripe: режим модели
      ctx.fillStyle = S.mode === 'DEGRADED' ? '#D6443A' : S.mode === 'SLIP' ? `rgb(${WET})` : S.gnssOff ? `rgb(${WARN})` : `rgb(${OK})`;
      ctx.fillRect(left, bottom - 0.62 * ppm, right - left, 0.2 * ppm);
      // window band with pillars
      const wTop = top + 0.45 * ppm, wH = 1.1 * ppm, w0 = -TL / 2 + 1.35, w1 = TL / 2 - 1.35;
      ctx.fillStyle = P.glass; ctx.fillRect(X(s + w0), wTop, (w1 - w0) * ppm, wH);
      ctx.fillStyle = P.body;
      for (let m = w0; m <= w1 + 1e-6; m += (w1 - w0) / 8) ctx.fillRect(X(s + m) - 0.1 * ppm, wTop, 0.2 * ppm, wH);
      // windshields
      ctx.fillStyle = P.glass;
      ctx.fillRect(left, top + 0.25 * ppm, 1.25 * ppm, 1.5 * ppm);
      ctx.fillRect(right - 1.25 * ppm, top + 0.25 * ppm, 1.25 * ppm, 1.5 * ppm);
      // doors
      DOORS.forEach(d => {
        const dx = X(s + d - 0.7), dw = 1.4 * ppm, dt = top + 0.35 * ppm, dh = bottom - 0.15 * ppm - dt;
        ctx.fillStyle = P.body; ctx.fillRect(dx - 0.08 * ppm, dt - 0.08 * ppm, dw + 0.16 * ppm, dh + 0.1 * ppm);
        ctx.fillStyle = P.door; ctx.fillRect(dx, dt, dw, dh);
        ctx.fillStyle = P.body; ctx.fillRect(dx + dw / 2 - 0.05 * ppm, dt, 0.1 * ppm, dh);
      });
      ctx.restore();
      ctx.strokeStyle = P.bodyStroke; ctx.lineWidth = 1; ctx.stroke(body);

      // lights (the tram runs to the right)
      const fx = right - 0.35 * ppm, bxL = left + 0.35 * ppm, ly = bottom - 0.45 * ppm;
      const lightsOn = night;
      if (lightsOn) {
        const len = 40 * ppm, k = 1;
        ctx.save();
        ctx.filter = `blur(${Math.max(6, 0.7 * ppm)}px)`;
        const bg = ctx.createLinearGradient(fx, 0, fx + len, 0);
        bg.addColorStop(0, `rgba(255,236,196,${0.55 * k})`);
        bg.addColorStop(0.15, `rgba(255,236,196,${0.3 * k})`);
        bg.addColorStop(0.45, `rgba(255,236,196,${0.12 * k})`);
        bg.addColorStop(0.75, `rgba(255,236,196,${0.04 * k})`);
        bg.addColorStop(1, 'rgba(255,236,196,0)');
        ctx.fillStyle = bg;
        ctx.beginPath();
        ctx.moveTo(fx, ly - 0.1 * ppm);
        ctx.quadraticCurveTo(fx + len * 0.5, ly - 1.4 * ppm, fx + len, railY - 2.8 * ppm);
        ctx.lineTo(fx + len, railY + 0.5 * ppm);
        ctx.quadraticCurveTo(fx + len * 0.4, railY + 0.2 * ppm, fx, ly + 0.15 * ppm);
        ctx.closePath(); ctx.fill();
        ctx.restore();
        const px = fx + len * 0.3, pr = len * 0.55;
        ctx.save(); ctx.translate(px, railY); ctx.scale(1, 0.06);
        const pool = ctx.createRadialGradient(0, 0, 0, 0, 0, pr);
        pool.addColorStop(0, `rgba(255,236,196,${0.22 * k})`); pool.addColorStop(0.5, `rgba(255,236,196,${0.07 * k})`); pool.addColorStop(1, 'rgba(255,236,196,0)');
        ctx.fillStyle = pool; ctx.fillRect(-pr, -pr, pr * 2, pr * 2);
        ctx.restore();
      }
      ctx.fillStyle = lightsOn ? '#fff6e0' : 'rgba(255,243,212,0.7)'; ctx.beginPath(); ctx.ellipse(fx - 0.04 * ppm, ly, Math.max(1, 0.06 * ppm), Math.max(1.5, 0.1 * ppm), 0, 0, Math.PI * 2); ctx.fill();
      if (lightsOn) {
        const tg = ctx.createRadialGradient(bxL, ly, 0, bxL, ly, 0.6 * ppm);
        tg.addColorStop(0, 'rgba(214,68,58,0.6)'); tg.addColorStop(1, 'rgba(214,68,58,0)');
        ctx.fillStyle = tg; ctx.fillRect(bxL - 0.6 * ppm, ly - 0.6 * ppm, 1.2 * ppm, 1.2 * ppm);
      }
      ctx.fillStyle = '#d6443a'; ctx.beginPath(); ctx.arc(bxL, ly, Math.max(1.5, 0.1 * ppm), 0, Math.PI * 2); ctx.fill();
      // GNSS antennas: master (links go here) and rover
      for (const [a, big] of [[ANT_MASTER, true], [ANT_ROVER, false]]) {
        ctx.fillStyle = P.bogie; ctx.fillRect(X(s + a) - 1, top - 0.35 * ppm, 2, 0.35 * ppm);
        ctx.beginPath(); ctx.arc(X(s + a), top - 0.35 * ppm, Math.max(big ? 2 : 1.6, (big ? 0.1 : 0.08) * ppm), 0, Math.PI * 2); ctx.fill();
      }

      // two bogies (front on the right) and wheels, each turning with its own wheel speed
      const wr = 0.36 * ppm, wy = railY - 0.14 * ppm - wr;
      [[HALF, 0], [-HALF, 1]].forEach(([bp, b]) => {
        const st = (S.bog || [])[b] || {};
        ctx.fillStyle = P.bogie;
        ctx.fillRect(X(s + bp - 1.35), bottom - 0.02 * ppm, 2.7 * ppm, 0.3 * ppm);
        [-0.95, 0.95].forEach(w => {
          const wx = X(s + bp + w), ang = (S.wheelAng || [0, 0])[b];
          if (st.spin) {   // буксует: синий ореол
            ctx.strokeStyle = `rgba(${WET},0.55)`; ctx.lineWidth = Math.max(1.5, 0.12 * ppm);
            ctx.beginPath(); ctx.arc(wx, wy, wr + Math.max(3, 0.22 * ppm), 0, Math.PI * 2); ctx.stroke();
          }
          ctx.fillStyle = P.bogie; ctx.beginPath(); ctx.arc(wx, wy, wr, 0, Math.PI * 2); ctx.fill();
          if (st.locked) { ctx.strokeStyle = '#D6443A'; ctx.lineWidth = Math.max(1.5, 0.08 * ppm); ctx.beginPath(); ctx.arc(wx, wy, wr, 0, Math.PI * 2); ctx.stroke(); }
          ctx.strokeStyle = st.locked ? '#D6443A' : P.hub; ctx.lineWidth = Math.max(1, 0.06 * ppm);
          ctx.beginPath();
          for (let k = 0; k < 2; k++) {
            const t = ang + k * Math.PI / 2;
            ctx.moveTo(wx - Math.cos(t) * wr * 0.75, wy - Math.sin(t) * wr * 0.75);
            ctx.lineTo(wx + Math.cos(t) * wr * 0.75, wy + Math.sin(t) * wr * 0.75);
          }
          ctx.stroke();
          ctx.fillStyle = st.locked ? '#D6443A' : P.hub; ctx.beginPath(); ctx.arc(wx, wy, wr * 0.22, 0, Math.PI * 2); ctx.fill();
          // юз: искры из-под колеса назад
          if (st.skid && !S.paused && Math.random() < 0.5) SKID.push({ x: wx - wr * 0.6, y: railY - 0.12 * ppm, vx: -(0.6 + Math.random() * 1.6) * ppm, vy: -(0.2 + Math.random() * 0.9) * ppm, life: 0.18 + Math.random() * 0.25, age: 0 });
        });
      });
      ctx.save(); ctx.globalCompositeOperation = night ? 'lighter' : 'source-over';
      for (let i = SKID.length - 1; i >= 0; i--) {
        const p = SKID[i]; p.age += sdt;
        if (p.age > p.life) { SKID.splice(i, 1); continue; }
        p.vy += 5 * ppm * sdt; p.x += p.vx * sdt; p.y += p.vy * sdt;
        ctx.fillStyle = `rgba(245,170,70,${0.9 * (1 - p.age / p.life)})`;
        ctx.fillRect(p.x, p.y, 1.8, 1.8);
      }
      ctx.restore();
    }

    // подписи под тележками: показание датчика и решение модели (принята / отброшена / исключена)
    function drawBogieLabels(S, ppm, X, RY, tilt, railY, mobile, night) {
      const bog = S.bog || [];
      if (!bog.length) return;
      const fs = mobile ? 10 : 11;
      [[HALF, 0], [-HALF, 1]].forEach(([bp, b]) => {
        const g = bog[b]; if (!g) return;
        const m = S.s + bp, x = X(S.s) + Math.cos(tilt) * (X(m) - X(S.s)), y = RY(m) + 1.25 * ppm + (mobile ? 10 : 12);
        // подписи расходятся от середины вагона: задняя - влево, передняя - вправо
        const align = b ? 'right' : 'left', ax = b ? x + (mobile ? 10 : 0.9 * ppm) : x - (mobile ? 10 : 0.9 * ppm);
        ctx.textAlign = align; ctx.textBaseline = 'middle';
        ctx.font = `${fs}px ${FONT}`;
        ctx.fillStyle = night ? 'rgba(238,241,244,0.6)' : 'rgba(0,0,0,0.5)';
        ctx.fillText(mobile ? g.kmhTxt : `${b ? 'задняя' : 'передняя'} · ${g.kmhTxt}`, ax, y);
        ctx.font = `500 ${fs}px ${FONT}`;
        ctx.fillStyle = g.col || (night ? 'rgba(238,241,244,0.6)' : 'rgba(0,0,0,0.5)');
        ctx.fillText(g.label, ax, y + fs + 3);
      });
      ctx.textBaseline = 'alphabetic'; ctx.textAlign = 'left';
    }

    return { draw, resize, get size() { return [W, H]; } };
  }
  return { create, PAL, TL, HALF };
})();
if (typeof module !== 'undefined') module.exports = TramScene;
