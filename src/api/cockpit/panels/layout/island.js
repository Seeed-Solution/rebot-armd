// Floating simple layout: keep the centred status island clear of the safety capsule (E-stop)
// and the preset pills. Narrow landscape screens and long labels would otherwise slide it
// over the E-stop button.
(() => {
  const body = document.body, tb = document.querySelector('.topbar');
  const safety = document.querySelector('.safety'), presets = document.querySelector('.presets');
  if (!tb || !safety) return;
  const hit = (a, b) => !(a.right <= b.left || b.right <= a.left || a.bottom <= b.top || b.bottom <= a.top);
  let busy = false;
  const root = document.documentElement, hud = document.querySelector('.hud');
  const shown = (el) => { if (!el) return null; const r = el.getBoundingClientRect(); return r.width && r.height ? r : null; };
  /* simple landscape: the HUD band between the thumb controls, the recognition popover above the joystick row,
     the pose card only when its smallest canvas still ends above the left joystick (or the pose toggle) */
  function placeThumbs() {
    body.classList.remove('pc-nofit');
    const L = ['pad-sxy', 'btn-pose'].map((id) => shown(document.getElementById(id))).filter(Boolean);
    const R = ['pad-srot', 'pad-sz', 'pad-sgrip', 's-grip-readout'].map((id) => shown(document.getElementById(id))).filter(Boolean);
    const W = innerWidth, H = innerHeight, gap = 12;
    const bandL = L.length ? Math.max(...L.map((r) => r.right)) + gap : gap;
    const bandR = R.length ? W - Math.min(...R.map((r) => r.left)) + gap : gap;
    root.style.setProperty('--band-l', Math.round(bandL) + 'px');
    root.style.setProperty('--band-r', Math.round(bandR) + 'px');
    const hr = shown(hud);
    root.style.setProperty('--hud-h', Math.round(hr ? hr.height : 28) + 'px');
    const pc = document.getElementById('pose-card'), pr = shown(pc);
    const lTop = L.length ? Math.min(...L.map((r) => r.top)) : H;
    if (pr && pr.bottom + 6 > lTop) body.classList.add('pc-nofit');
    // popover: centred between the left column and the presets, above every joystick / slider under it
    const leftCol = [document.querySelector('.safety'), body.classList.contains('pc-nofit') ? null : pc, document.getElementById('view-switch'),
      root.style.getPropertyValue('--tb-below') ? tb : null]
      .map(shown).filter(Boolean).reduce((m, r) => Math.max(m, r.right), 0);
    const presets = shown(document.querySelector('.presets'));
    const lo = leftCol + 8, hi = (presets ? presets.left : W) - 8;
    const rowTop = Math.min(H, ...[...L, ...R, hr].filter((r) => r && r.right > lo && r.left < hi).map((r) => r.top));
    root.style.setProperty('--pop-l', Math.round((lo + hi) / 2) + 'px');
    root.style.setProperty('--pop-w', Math.max(160, Math.round(hi - lo)) + 'px');
    root.style.setProperty('--pop-b', Math.round(H - rowTop + 8) + 'px');
    // recognition panel (#s-vis): above its chip, inside the band between the thumb controls when that is wide enough
    // (else the popover span above the joystick row), its top below whatever sits above it in that span
    // Candidates in order: the band (one column, 400 px), the popover span (one column), the popover span in three
    // columns (short screens); the first whose height fits the panel wins, else the tallest.
    const chip = shown(document.getElementById('btn-vis'));
    const bandW = W - bandL - bandR, span = hi - lo;
    const cands = [];
    if (bandW >= 300) cands.push({ l: (bandL + W - bandR) / 2, w: Math.min(400, bandW), b: H - (chip ? chip.top : H - 16) + 8, need: 400 });
    if (span >= 300) cands.push({ l: (lo + hi) / 2, w: Math.min(400, span), b: H - rowTop + 8, need: 400 });
    if (span >= 520) cands.push({ l: (lo + hi) / 2, w: Math.min(680, span), b: H - rowTop + 8, need: 200, wide: true });
    if (!cands.length) cands.push({ l: (lo + hi) / 2, w: Math.max(160, span), b: H - rowTop + 8, need: 0 });
    const blockers = [document.querySelector('.safety'), body.classList.contains('pc-nofit') ? null : pc, document.getElementById('view-switch'), tb, document.querySelector('.presets')].map(shown).filter(Boolean);
    for (const c of cands) {
      const topLim = blockers.filter((r) => r.right > c.l - c.w / 2 && r.left < c.l + c.w / 2).reduce((m, r) => Math.max(m, r.bottom), 8) + 8;
      c.maxh = H - c.b - topLim;
    }
    const sv = cands.find((c) => c.maxh >= c.need) || cands.reduce((a, c) => (c.maxh > a.maxh ? c : a));
    root.style.setProperty('--sv-l', Math.round(sv.l) + 'px');
    root.style.setProperty('--sv-w', Math.round(sv.w) + 'px');
    root.style.setProperty('--sv-b', Math.round(sv.b) + 'px');
    root.style.setProperty('--sv-maxh', Math.max(120, Math.round(sv.maxh)) + 'px');
    document.getElementById('s-vis')?.classList.toggle('wide', !!sv.wide);
  }
  function place() {
    if (busy) return; busy = true;
    try {
      body.classList.remove('tb-compact-1', 'tb-compact-2', 'tb-compact-3');
      root.style.removeProperty('--tb-below');
      if (body.classList.contains('ui-simple') && getComputedStyle(tb).position === 'fixed') placeThumbs();
      else body.classList.remove('pc-nofit');
      tb.style.left = ''; tb.style.top = ''; tb.style.transform = '';
      if (!body.classList.contains('ui-simple') || getComputedStyle(tb).position !== 'fixed') return;
      const gap = 8;
      for (let level = 0; level <= 3; level++) {
        if (level) body.classList.add('tb-compact-' + level);
        tb.style.left = ''; tb.style.transform = '';
        const s = safety.getBoundingClientRect(), t = tb.getBoundingClientRect();
        if (!hit(s, t)) return;
        const right = presets && presets.getBoundingClientRect().width ? presets.getBoundingClientRect().left : innerWidth;
        const space = right - s.right - 2 * gap;
        if (t.width <= space) {
          tb.style.left = Math.round(s.right + gap + (space - t.width) / 2) + 'px';
          tb.style.transform = 'none';
          return;
        }
      }
      // no room beside the capsule: put the island under it, left-aligned, and move the view switch and the pose card
      // (they sit under the capsule) below the island
      const s = safety.getBoundingClientRect();
      tb.style.left = Math.round(s.left) + 'px';
      tb.style.top = Math.round(s.bottom + gap) + 'px';
      tb.style.transform = 'none';
      root.style.setProperty('--tb-below', Math.round(tb.getBoundingClientRect().height + gap) + 'px');
      placeThumbs();
    } finally { busy = false; }
  }
  const ro = new ResizeObserver(() => requestAnimationFrame(place));
  [safety, tb, presets, hud].forEach(el => el && ro.observe(el));
  const baseClass = () => body.className.replace(/\b(tb-compact-\d|pc-nofit|vis-open)\b/g, '').replace(/\s+/g, ' ').trim();
  let lastClass = baseClass();
  new MutationObserver(() => { const c = baseClass(); if (c !== lastClass) { lastClass = c; requestAnimationFrame(place); } })
    .observe(body, { attributes: true, attributeFilter: ['class'] });
  addEventListener('resize', () => requestAnimationFrame(place));
  requestAnimationFrame(place);
})();
