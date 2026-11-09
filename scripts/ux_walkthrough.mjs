const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
import fs from 'fs';
import path from 'path';

const outDir = '/tmp/ux_walkthrough';
fs.mkdirSync(outDir, { recursive: true });

async function run() {
  console.log('[*] Launching Chromium (iPad landscape 1024x768)...');
  const browser = await chromium.launch({
    executablePath: process.env.CHROME || undefined,
    headless: true,
  });

  const context = await browser.newContext({
    viewport: { width: 1024, height: 768 },
    deviceScaleFactor: 2,
    userAgent: 'Mozilla/5.0 (iPad; CPU OS 17_0 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) Version/17.0 Mobile/15E148 Safari/604.1'
  });

  const page = await context.newPage();
  const errors = [];
  page.on('pageerror', (err) => {
    console.error('[-] Page Error:', err.message);
    errors.push(err.message);
  });

  console.log('[*] Navigating to http://127.0.0.1:8765/ ...');
  await page.goto('http://127.0.0.1:8765/', { waitUntil: 'networkidle' });
  await page.waitForTimeout(1000);

  // 1. Verify title
  const title = await page.title();
  console.log('[+] Page Title:', title);

  // 2. Check Cockpit elements
  const hasTriggerBtn = await page.$('#btn-trigger') !== null;
  const hasDetBtn = await page.$('#btn-det') !== null;
  const hasPill = await page.$('#fps-pill') !== null;
  const hasCalibBtn = await page.$('#btn-calib') !== null;
  const hasCalibDrawer = await page.$('#calib') !== null;

  console.log('[+] Element check:');
  console.log('    #btn-trigger (单次识别):', hasTriggerBtn);
  console.log('    #btn-det (识别开关):', hasDetBtn);
  console.log('    #fps-pill (状态胶囊):', hasPill);
  console.log('    #btn-calib (标定入口):', hasCalibBtn);
  console.log('    #calib (标定抽屉遮罩):', hasCalibDrawer);

  const pillText = await page.$eval('#fps-pill', (el) => el.textContent.trim());
  console.log('    #fps-pill text:', pillText);

  // 3. Take main cockpit screenshot (Simple mode)
  await page.screenshot({ path: path.join(outDir, '01_main_cockpit_simple.png') });
  console.log('[+] Saved 01_main_cockpit_simple.png');

  // 4. Switch to Expert UI Mode
  console.log('[*] Switching to Expert UI Mode via setUi("expert")...');
  await page.evaluate(() => setUi('expert'));
  await page.waitForTimeout(500);

  await page.screenshot({ path: path.join(outDir, '02_main_cockpit_expert.png') });
  console.log('[+] Saved 02_main_cockpit_expert.png');

  // 5. Open Calibration Drawer (#btn-calib)
  console.log('[*] Clicking #btn-calib to open drawer...');
  await page.click('#btn-calib');
  await page.waitForTimeout(600);

  const isDrawerVisible = await page.evaluate(() => {
    const d = document.getElementById('calib');
    return d && !d.hidden && window.getComputedStyle(d).display !== 'none';
  });
  console.log('[+] Calibration Drawer opened visible:', isDrawerVisible);

  const countText = await page.evaluate(() => {
    const el = document.getElementById('calib-count');
    return el ? el.textContent : '';
  });
  console.log('[+] Drawer Sample Count text:', countText);

  await page.screenshot({ path: path.join(outDir, '03_calibration_drawer_open.png') });
  console.log('[+] Saved 03_calibration_drawer_open.png');

  // 6. Test Close Button on Calibration Drawer
  console.log('[*] Clicking #calib-close...');
  await page.click('#calib-close');
  await page.waitForTimeout(400);

  const isDrawerClosed = await page.evaluate(() => {
    const d = document.getElementById('calib');
    return d && (d.hidden || window.getComputedStyle(d).display === 'none');
  });
  console.log('[+] Calibration Drawer closed:', isDrawerClosed);

  // 7. Inject simulated ArUco 6-DoF detection to test 3D AR rendering on Canvas
  console.log('[*] Injecting synthetic ArUco marker to test AR Overlay rendering...');
  await page.evaluate(() => {
    if (typeof S !== 'undefined') {
      S.detections = [{
        x0: 0.35, y0: 0.30, x1: 0.55, y1: 0.50,
        cx: 0.45, cy: 0.40, w: 0.20, h: 0.20,
        label: 'ID:0', score: 1.0,
        aruco: {
          id: 0,
          corners: [[0.35, 0.30], [0.55, 0.30], [0.55, 0.50], [0.35, 0.50]],
          t: [0.045, -0.012, 0.354],
          rpy: [-1.2, 3.4, 45.0],
          error: 0.001
        }
      }];
      S.detLastReceivedAt = performance.now();
      S.detHoldUntil = performance.now() + 10000;
      S.detSource = 'mock_aruco';
      if (typeof drawOverlay === 'function') {
        drawOverlay();
      }
    }
  });

  await page.waitForTimeout(200);
  await page.screenshot({ path: path.join(outDir, '04_aruco_3d_ar_overlay.png') });
  console.log('[+] Saved 04_aruco_3d_ar_overlay.png');

  await browser.close();
  console.log('\n[=== UX Walkthrough Finished Successfully ===]');
  console.log('Total Page Errors:', errors.length);
  if (errors.length > 0) {
    console.error('Errors encountered:', errors);
    process.exit(1);
  }
}

run().catch((err) => {
  console.error('Fatal walkthrough error:', err);
  process.exit(1);
});
