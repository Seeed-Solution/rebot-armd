const { chromium } = require('playwright');
const path = require('path');
const fs = require('fs');

(async () => {
  const browser = await chromium.launch({
    executablePath: process.env.CHROME || undefined,
    headless: true,
    args: ['--no-sandbox', '--disable-setuid-sandbox']
  });

  const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });

  const consoleLogs = [];
  const consoleErrors = [];
  page.on('console', msg => {
    const text = msg.text();
    consoleLogs.push(text);
    if (msg.type() === 'error') consoleErrors.push(text);
  });

  console.log('Navigating to Single-Origin Gateway: http://127.0.0.1:8765/ ...');
  await page.goto('http://127.0.0.1:8765/', { waitUntil: 'networkidle', timeout: 15000 });

  // Wait 4 seconds for telemetry stream and heartbeat
  await page.waitForTimeout(4000);

  // Check state on page
  const state = await page.evaluate(() => {
    return {
      wsReady: typeof S !== 'undefined' && S.ws ? S.ws.readyState : 0,
      wsCount: typeof S !== 'undefined' ? S.wsCount : 0,
      hz: typeof S !== 'undefined' ? S.hz : 0,
      hbCount: typeof S !== 'undefined' && S.hb ? S.hb.n : 0,
      detSource: typeof S !== 'undefined' ? S.detSource : null,
      videoMode: typeof V !== 'undefined' ? V.mode : null,
      videoHost: typeof CFG !== 'undefined' && CFG.video ? CFG.video.host : null,
      dof: typeof S !== 'undefined' ? S.dof : null,
      title: document.title,
    };
  });

  console.log('Client State Extracted:', JSON.stringify(state, null, 2));

  // Take screenshot
  const shotDir = path.resolve(__dirname, 'artifacts');
  fs.mkdirSync(shotDir, { recursive: true });
  const shotPath = path.join(shotDir, 'playwright_single_origin_gateway_verified.png');
  await page.screenshot({ path: shotPath, fullPage: true });
  console.log('Screenshot saved to:', shotPath);

  await browser.close();

  // Assertions
  if (state.wsReady !== 1) {
    console.error('FAIL: WebSocket telemetry not connected (readyState != 1) on port 8765!');
    process.exit(1);
  }
  if (state.hbCount < 1) {
    console.error('FAIL: No heartbeats sent!');
    process.exit(1);
  }
  if (state.videoHost !== '127.0.0.1:8765') {
    console.error('FAIL: videoHost does not match single origin 127.0.0.1:8765! Actual:', state.videoHost);
    process.exit(1);
  }

  console.log('SUCCESS: All single-origin gateway end-to-end assertions passed!');
  process.exit(0);
})().catch(err => {
  console.error('Error during test:', err);
  process.exit(1);
});
