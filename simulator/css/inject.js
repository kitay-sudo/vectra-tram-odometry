// Вставляет собранный CSS в simulator/index.html между маркерами /* tw:begin */ и /* tw:end */.
const fs = require('fs');
const [css, html] = process.argv.slice(2);
const src = fs.readFileSync(html, 'utf8');
const out = fs.readFileSync(css, 'utf8').trim();
const a = src.indexOf('/* tw:begin */'), b = src.indexOf('/* tw:end */');
if (a < 0 || b < a) { console.error('маркеры /* tw:begin */ … /* tw:end */ не найдены'); process.exit(1); }
const res = src.slice(0, a + '/* tw:begin */'.length) + '\n' + out.replace(/\/\*[^]*?\*\//g, '') + '\n' + src.slice(b);
fs.writeFileSync(html, res);
console.log(`tailwind: ${out.length} байт CSS вставлено`);
