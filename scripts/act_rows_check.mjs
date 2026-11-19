// Layout check of the action-button rows (.act-row, see tidyActs in src/api/index.html), evaluated in the page:
//   import { actRowsCheck } from './act_rows_check.mjs';  const bad = await page.evaluate(actRowsCheck, '#cwiz-panel');
// For every visible row under the selector: all buttons the same height (+-1 px); no button narrower than 40 % of the row
// unless the row shows >= 3 columns on a viewport wider than 480 px; no label on more than 2 lines; at most one
// full-width primary and no lone half-width button (an odd last button spans both columns). Returns a list of problems.
export function actRowsCheck(sel) {
  const out = [];
  const lines = (b) => { const r = document.createRange(); r.selectNodeContents(b); return new Set([...r.getClientRects()].filter((x) => x.width > 0.5).map((x) => Math.round(x.top))).size; };
  for (const root of document.querySelectorAll(sel)) {
    if (root.closest('[hidden]')) continue;
    for (const row of root.querySelectorAll('.act-row')) {
      const rr = row.getBoundingClientRect();
      if (!(rr.width > 0 && rr.height > 0)) continue;
      const bs = [...row.children].filter((b) => !b.hidden && b.getBoundingClientRect().width > 0);
      const rects = bs.map((b) => b.getBoundingClientRect());
      const name = (b) => (b.id || b.textContent.trim()).slice(0, 24);
      const hs = rects.map((r) => r.height);
      if (Math.max(...hs) - Math.min(...hs) > 1) out.push(`heights ${hs.map(Math.round).join('/')} in [${bs.map(name).join(', ')}]`);
      const cols = new Set(rects.map((r) => Math.round(r.left))).size;
      const wideOk = cols >= 3 && window.innerWidth > 480;
      bs.forEach((b, i) => {
        if (!wideOk && rects[i].width < rr.width * 0.4) out.push(`"${name(b)}" ${Math.round(rects[i].width)} px of ${Math.round(rr.width)} px`);
        const n = lines(b);
        if (n > 2) out.push(`"${name(b)}" label on ${n} lines`);
      });
      // a half-width button alone on its grid line
      const byTop = new Map();
      rects.forEach((r, i) => { const k = Math.round(r.top); byTop.set(k, [...(byTop.get(k) || []), i]); });
      for (const idx of byTop.values()) if (idx.length === 1 && rects[idx[0]].width < rr.width * 0.9) out.push(`"${name(bs[idx[0]])}" alone at half width`);
    }
  }
  return out;
}
