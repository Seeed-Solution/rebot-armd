/* =================================================================
   Boot
   ================================================================= */
{ const sp = store('rebot.speed'); setSpeed(sp != null && /^[0-2]$/.test(sp) ? Number(sp) : 1); }
{
  setUi(QS.get('ui') || 'simple'); // always opens in simple mode; ?ui=expert deep-links
  const st = store('rebot.step'); setStep(st != null && /^[0-2]$/.test(st) ? Number(st) : 1);
  $('c-rmax').textContent = Math.round(CFG.cart.reachMax * 1000);
}
schemaReady = loadSchema();
loadOwnerDiagnostics();
loadAuthStatus();
setInterval(() => { if (S.auth.role === 'operator') { renderOpBadge(); opCheckExpiry(); } }, 10000); // minutes left on the badge, renewal before the end of a flow
initSandbox();
connectWS();
connectDetectionWS();
initDetToggle();
initVisionMode();
initDepth();
initSVis();
initExpertMode();
initCalib();
initCwiz();
videoStart();
syncControlInputsUi(pageHasControl());
requestAnimationFrame(frame);

/* i18n: re-render dynamic UI on language change */
i18nHooks.push(
  renderJogRows, renderGripCol, renderPoseUi, renderOpBadge, renderPreemptModal, renderTakeModal,
  () => setMode(S.ctl, S.input),
  () => { if (lastHint) videoHint(...lastHint); },
  () => {
    $('btn-rec').textContent = S.rec.active ? t('rec.stop') : t('rec.start');
    $('rec-state').textContent = S.rec.active ? t('rec.recording') : t('rec.idle');
    renderRecLast();
  },
);
$('btn-lang').addEventListener('click', () => setLang(LANG === 'zh' ? 'en' : 'zh'));
applyI18n();
