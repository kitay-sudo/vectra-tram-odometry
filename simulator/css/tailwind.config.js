// Tailwind v3 (как Play CDN https://cdn.tailwindcss.com): статический CSS для работы без сети.
// Сборка: bash simulator/css/build_css.sh (Docker, node:22-alpine).
module.exports = {
  content: ['../index.html'],
  theme: { extend: {} },
  plugins: [],
};
