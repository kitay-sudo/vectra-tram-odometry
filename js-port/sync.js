// Копии порта для страницы: js-port/{est,runner}.js -> simulator/js/ (байт в байт).
//   node js-port/sync.js        (после любой правки est.js или runner.js; check.sh сверяет копии)
const fs = require('fs'), path = require('path');
const src = __dirname, dst = path.join(__dirname, '..', 'simulator', 'js');
fs.mkdirSync(dst, { recursive: true });
for (const f of ['est.js', 'runner.js']) {
  fs.copyFileSync(path.join(src, f), path.join(dst, f));
  console.log('скопировано', f, '->', path.relative(process.cwd(), path.join(dst, f)));
}
