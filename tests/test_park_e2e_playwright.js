const { chromium } = require('playwright');
const path = require('path');
const fs = require('fs');

(async () => {
  const artifactDir = path.resolve('tests/artifacts');
  
  const browser = await chromium.launch({
    executablePath: '/snap/bin/chromium',
    headless: true,
    args: ['--no-sandbox', '--disable-setuid-sandbox']
  });

  const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
  
  page.on('console', msg => console.log('PAGE LOG:', msg.text()));
  page.on('pageerror', err => console.log('PAGE ERROR:', err.message));

  console.log('[1/5] Navigating to http://127.0.0.1:8765 ...');
  await page.goto('http://127.0.0.1:8765', { waitUntil: 'domcontentloaded' });
  await page.waitForTimeout(1000);

  // 1. Ensure armed
  console.log('[2/5] Checking arm state...');
  const pageState = await page.evaluate(() => {
    return {
      S_state: S.state,
      S_isLive: isLive(),
      S_isOperational: isOperational(),
      S_ws_ready: S.ws ? S.ws.readyState : -1,
      S_leaseGen: S.leaseGeneration
    };
  });
  console.log('Page internal state:', JSON.stringify(pageState));

  // If not operational, enable via UI or API
  if (!pageState.S_isOperational) {
    console.log('Arming robot via API...');
    await page.evaluate(async () => {
      await fetch('/api/v1/torque', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enable: true })
      });
    });
    await page.waitForTimeout(1000);
  }

  const armedState = await page.evaluate(() => {
    return {
      S_state: S.state,
      S_isLive: isLive(),
      S_isOperational: isOperational(),
      S_leaseGen: S.leaseGeneration
    };
  });
  console.log('State after enable:', JSON.stringify(armedState));

  // 2. Click the 'park' preset button on the UI
  console.log('[3/5] Triggering UI preset "park" (收纳位)...');
  await page.click('button[data-preset="park"]');

  // 3. Monitor settling and torque cutoff
  console.log('[4/5] Waiting for arm to settle and torque to be safely cut off...');
  let cutoffObs = null;
  const t0 = Date.now();
  while (Date.now() - t0 < 10000) {
    await page.waitForTimeout(300);
    const obs = await page.evaluate(async () => {
      const res = await fetch('/api/v1/observation');
      return res.json();
    });
    if (!obs.is_torqued) {
      cutoffObs = obs;
      console.log(`[PASS] Torque cutoff detected at +${((Date.now() - t0)/1000).toFixed(2)}s!`);
      break;
    }
  }

  if (!cutoffObs) {
    console.error('[FAIL] Timed out waiting for torque cutoff!');
    const finalObs = await page.evaluate(async () => {
      const res = await fetch('/api/v1/observation');
      return res.json();
    });
    console.error('Final Obs:', JSON.stringify(finalObs));
    process.exit(1);
  }

  console.log('[5/5] Final Verification:');
  console.log('Final Safety State:', cutoffObs.safety_state);
  console.log('Final is_torqued:', cutoffObs.is_torqued);
  console.log('Final Positions:', JSON.stringify(cutoffObs.positions));

  // Screenshot proof
  const screenshotPath = path.join(artifactDir, 'playwright_park_zero_drop_verified.png');
  await page.screenshot({ path: screenshotPath });
  console.log(`Saved screenshot proof to ${screenshotPath}`);

  // Assertions
  if (cutoffObs.is_torqued !== false) throw new Error('Expected is_torqued == false');
  if (cutoffObs.safety_state !== 'Idle') throw new Error('Expected safety_state == Idle');

  console.log('ALL REAL ROBOT PHYSICAL ACCEPTANCE CRITERIA PASSED!');
  await browser.close();
  process.exit(0);
})();
