let LANG = (() => {
  const saved = store('lang');
  if (saved === 'zh' || saved === 'en') return saved;
  return /^zh/i.test(navigator.language || '') ? 'zh' : 'en';
})();
// '@7' when the arm reports 7 DOF: keys with a 7-DOF variant resolve to it.
let I18N_VARIANT = '';
// true while the camera-view cartesian frame is active: '@view' keys win over '@7' and the plain key
let I18N_VIEW = false;
function t(key, vars) {
  let s = I18N_VIEW ? (I18N[LANG][key + '@view'] ?? I18N.zh[key + '@view']) : undefined;
  if (s === undefined || s === '') s = I18N_VARIANT && (I18N[LANG][key + I18N_VARIANT] ?? I18N.zh[key + I18N_VARIANT]);
  if (s === undefined || s === '') s = I18N[LANG][key];
  if (s === undefined) s = I18N.zh[key];
  if (s === undefined) return key;
  return vars ? s.replace(/\{(\w+)\}/g, (m, k) => (k in vars ? vars[k] : m)) : s;
}
const i18nHooks = [];
function ikText(m) { return m && I18N.zh[m] !== undefined ? t(m) : m; }
function applyI18n() {
  document.documentElement.lang = LANG === 'zh' ? 'zh-CN' : 'en';
  document.title = t('doc.title');
  for (const el of document.querySelectorAll('[data-i18n]')) el.textContent = t(el.dataset.i18n);
  for (const [attr, ds] of [['title', 'i18nTitle'], ['aria-label', 'i18nAria'], ['placeholder', 'i18nPlaceholder']]) {
    for (const el of document.querySelectorAll('[data-i18n-' + attr.replace('aria-label', 'aria') + ']')) el.setAttribute(attr, t(el.dataset[ds]));
  }
  const lb = document.getElementById('btn-lang');
  if (lb) { lb.textContent = t('lang.btn'); lb.setAttribute('aria-label', t('lang.aria')); }
  for (const h of i18nHooks) h();
}
function setLang(l) {
  LANG = l === 'en' ? 'en' : 'zh'; store('lang', LANG);
  const tc = document.getElementById('toasts'); if (tc) tc.textContent = ''; // transient toasts are in the old language
  applyI18n();
}
