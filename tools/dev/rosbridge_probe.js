// Аудит режима «Живой ROS 2»: клиент rosbridge (protocol v2, JSON) из Node 22.
//
//   node tools/dev/rosbridge_probe.js [ws://127.0.0.1:19090] [seconds=60] [throttle_ms=0]
//
// Подписывается на выходы ноды и входы, считает частоту, размер JSON-кадра,
// задержку «вход (front_bogie) -> выход (/result/velocity) с меткой >= входа»,
// измеренную на стороне браузера-клиента (включает мост и websocket).
const fs = require('fs');
const path = require('path');
const url = process.argv[2] || 'ws://127.0.0.1:19090';
const DUR = +(process.argv[3] || 60) * 1000;
const THR = +(process.argv[4] || 0);
const TOPICS = [
  ['/result/velocity', 'tram_vehicle_msgs/msg/VelocitySensor'],
  ['/result/position', 'nav_msgs/msg/Odometry'],
  ['/result/acceleration', 'geometry_msgs/msg/AccelStamped'],
  ['/tram/estimator_status', 'tram_msgs/msg/EstimatorStatus'],
  ['/vehicle/front_bogie_velocity', 'tram_vehicle_msgs/msg/VelocitySensor'],
  ['/vehicle/rear_bogie_velocity', 'tram_vehicle_msgs/msg/VelocitySensor'],
  ['/vehicle/driver_position_cmd', 'tram_vehicle_msgs/msg/DriverControllerCommand'],
  ['/sensing/gnss/master/vel', 'geometry_msgs/msg/TwistStamped'],
  ['/sensing/gnss/master/fix', 'sensor_msgs/msg/NavSatFix'],
];
const st = Object.fromEntries(TOPICS.map(([t]) => [t, { n: 0, bytes: 0, first: null, last: null, sample: null }]));
const pendingIn = [];   // [stamp, wall_ms] входов front_bogie
const lat = [];
const stamp = h => h.stamp.sec + h.stamp.nanosec * 1e-9;
let t0 = null;

function connect(tries = 0) {
  const ws = new WebSocket(url);
  ws.onopen = () => {
    console.log('connected', url);
    t0 = Date.now();
    for (const [topic, type] of TOPICS) ws.send(JSON.stringify({ op: 'subscribe', topic, type, throttle_rate: THR, queue_length: 1 }));
    setTimeout(() => { ws.close(); report(); }, DUR);
  };
  ws.onerror = () => { if (tries < 40) setTimeout(() => connect(tries + 1), 1000); else { console.log('no bridge'); process.exit(2); } };
  ws.onmessage = ev => {
    const now = Date.now();
    const raw = typeof ev.data === 'string' ? ev.data : '';
    const m = JSON.parse(raw);
    if (m.op !== 'publish') { console.log('op', m.op, raw.slice(0, 200)); return; }
    const s = st[m.topic]; if (!s) return;
    s.n++; s.bytes += raw.length; s.first ??= now; s.last = now; s.sample ??= m.msg;
    if (m.topic === '/vehicle/front_bogie_velocity') pendingIn.push([stamp(m.msg.header), now]);
    if (m.topic === '/result/velocity') {
      const ts = stamp(m.msg.header);
      while (pendingIn.length && pendingIn[0][0] <= ts + 1e-6) {
        const [, w] = pendingIn.shift();
        if (!pendingIn.length || pendingIn[0][0] > ts) lat.push(now - w);
      }
    }
  };
}
function report() {
  const rows = {};
  for (const [t, s] of Object.entries(st)) {
    const dur = s.n > 1 ? (s.last - s.first) / 1000 : 0;
    rows[t] = { n: s.n, hz: dur ? +(s.n / dur).toFixed(1) : 0, avg_json_bytes: s.n ? Math.round(s.bytes / s.n) : 0, kbit_s: dur ? +(s.bytes * 8 / 1000 / dur).toFixed(1) : 0 };
  }
  lat.sort((a, b) => a - b);
  const q = p => (lat.length ? lat[Math.min(lat.length - 1, Math.floor(p * lat.length))] : null);
  const res = { url, throttle_ms: THR, seconds: DUR / 1000, topics: rows,
    latency_in_to_out_ms: { n: lat.length, p50: q(0.5), p95: q(0.95), max: lat.length ? lat[lat.length - 1] : null },
    samples: Object.fromEntries(Object.entries(st).map(([t, s]) => [t, s.sample])) };
  console.table(rows);
  console.log('задержка front_bogie -> /result/velocity на клиенте, мс:', res.latency_in_to_out_ms);
  const out = path.resolve(__dirname, '..', '..', 'out', 'sim', `rosbridge_probe_thr${THR}.json`);
  fs.writeFileSync(out, JSON.stringify(res, null, 1));
  console.log('->', out);
  process.exit(0);
}
connect();
