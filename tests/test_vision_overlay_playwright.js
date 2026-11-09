const { chromium } = require('playwright');
const path = require('path');
const fs = require('fs');
const { execSync } = require('child_process');

(async () => {
  const artifactDir = path.resolve(__dirname, 'artifacts');
  if (!fs.existsSync(artifactDir)) fs.mkdirSync(artifactDir, { recursive: true });

  let gitCommit = 'unknown';
  try {
    gitCommit = execSync('git rev-parse HEAD', { cwd: __dirname }).toString().trim();
  } catch (_) {}

  const browser = await chromium.launch({
    executablePath: process.env.CHROME || undefined,
    headless: true,
    args: ['--no-sandbox', '--disable-setuid-sandbox']
  });

  const viewports = [
    { name: 'tablet_landscape', width: 1024, height: 768 },
    { name: 'desktop', width: 1920, height: 1080 },
    { name: 'mobile', width: 390, height: 844 }
  ];

  const logLines = [];
  function log(...args) {
    console.log(...args);
    logLines.push(args.map(a => typeof a === 'object' ? JSON.stringify(a, null, 2) : String(a)).join(' '));
  }

  log('=====================================================================');
  log('Starting Playwright Vision Overlay Multi-Viewport First-Principle Test');
  log(`Repository Git Commit: ${gitCommit}`);
  log('=====================================================================');

  let globalMaxError = 0.0;
  let totalCheckedDimensions = 0;

  for (const vp of viewports) {
    const page = await browser.newPage({ viewport: { width: vp.width, height: vp.height } });
    const filePath = 'file://' + path.resolve(__dirname, '../src/api/index.html');
    await page.goto(filePath, { waitUntil: 'domcontentloaded' });

    const evalResult = await page.evaluate(() => {
      const vhint = document.getElementById('vhint');
      if (vhint) {
        vhint.style.display = 'none';
        vhint.classList.add('hide');
      }

      const v = document.getElementById('video-stream');
      Object.defineProperty(v, 'videoWidth', { value: 1280, configurable: true });
      Object.defineProperty(v, 'videoHeight', { value: 720, configurable: true });
      v.style.backgroundColor = '#0f172a';
      v.style.display = 'block';

      const box = document.getElementById('video-box');
      box.style.backgroundColor = '#020617';

      window.V.mode = 'webrtc';

      const bRect = box.getBoundingClientRect();
      const boxW = bRect.width;
      const boxH = bRect.height;

      const renderedBoxes = [];
      const renderedArcs = [];
      const origStrokeRect = CanvasRenderingContext2D.prototype.strokeRect;
      const origArc = CanvasRenderingContext2D.prototype.arc;

      CanvasRenderingContext2D.prototype.strokeRect = function(x, y, w, h) {
        renderedBoxes.push({ x, y, w, h });
        return origStrokeRect.apply(this, arguments);
      };
      CanvasRenderingContext2D.prototype.arc = function(x, y, radius, startAngle, endAngle) {
        renderedArcs.push({ x, y, radius });
        return origArc.apply(this, arguments);
      };

      const testDetections = window.parseDetections({
        data: {
          boxes: [
            [261.2, 375.9, 125.8, 45.4, 0.82, 'banana'],
            [10, 360, 40, 100, 0.90, 'cut_left'],
            [0.95, 0.5, 0.4, 0.2, 0.90, 'cut_right'],
            [640, 15, 100, 50, 0.88, 'cut_top'],
            [0.5, 0.95, 0.2, 0.3, 0.85, 'cut_bottom']
          ],
          resolution: [1280, 720]
        }
      });
      window.S.detections = testDetections;
      window.S.detSource = 'det_ws';
      window.S.detLastReceivedAt = performance.now();

      window.drawOverlay();

      CanvasRenderingContext2D.prototype.strokeRect = origStrokeRect;
      CanvasRenderingContext2D.prototype.arc = origArc;

      window.__overlayKeepAlive = setInterval(() => {
        window.S.detLastReceivedAt = performance.now();
        window.drawOverlay();
      }, 30);

      const crosshairCenters = renderedArcs.filter(a => Math.abs(a.radius - 1.5) < 0.1 || Math.abs(a.radius - 4) < 0.1);

      return {
        boxW,
        boxH,
        detectionsCount: testDetections.length,
        actualStrokeRects: renderedBoxes,
        actualCrosshairCenters: crosshairCenters
      };
    });

    const srcW = 1280;
    const srcH = 720;
    const boxW = evalResult.boxW;
    const boxH = evalResult.boxH;

    // 检查几何基线的有限性
    if (!Number.isFinite(boxW) || !Number.isFinite(boxH) || boxW <= 0 || boxH <= 0) {
      throw new Error(`${vp.name}: box dimensions not finite positive numbers: ${boxW}x${boxH}`);
    }

    const indepScale = Math.min(boxW / srcW, boxH / srcH);
    const indepRenderW = srcW * indepScale;
    const indepRenderH = srcH * indepScale;
    const indepOffsetX = (boxW - indepRenderW) / 2;
    const indepOffsetY = (boxH - indepRenderH) / 2;

    log(`[${vp.name}] Viewport (${vp.width}x${vp.height}):`);
    log(`  Independent Geometry: boxW=${boxW.toFixed(4)}, boxH=${boxH.toFixed(4)}, scale=${indepScale.toFixed(6)}, renderW=${indepRenderW.toFixed(4)}, renderH=${indepRenderH.toFixed(4)}, offset=(${indepOffsetX.toFixed(4)}, ${indepOffsetY.toFixed(4)})`);
    log(`  Captured Calls: strokeRects=${evalResult.actualStrokeRects.length}, crosshairArcs=${evalResult.actualCrosshairCenters.length}`);

    if (evalResult.detectionsCount !== 5) throw new Error(`${vp.name}: Expected 5 detections, got ${evalResult.detectionsCount}`);
    if (evalResult.actualStrokeRects.length !== 5) throw new Error(`${vp.name}: Expected 5 strokeRect calls, got ${evalResult.actualStrokeRects.length}`);
    if (evalResult.actualCrosshairCenters.length !== 10) throw new Error(`${vp.name}: Expected 10 crosshair arc calls, got ${evalResult.actualCrosshairCenters.length}`);

    // 1. 严格检查全部 5 次 strokeRect 的入参有限性
    for (let k = 0; k < evalResult.actualStrokeRects.length; k++) {
      const b = evalResult.actualStrokeRects[k];
      if (!Number.isFinite(b.x) || !Number.isFinite(b.y) || !Number.isFinite(b.w) || !Number.isFinite(b.h)) {
        throw new Error(`${vp.name}: StrokeRect ${k} contains non-finite values: ${JSON.stringify(b)}`);
      }
    }

    // 2. 严格检查全部 10 次 arc 的入参有限性，并验证每对 arc 中心严格重合
    for (let k = 0; k < evalResult.actualCrosshairCenters.length; k++) {
      const a = evalResult.actualCrosshairCenters[k];
      if (!Number.isFinite(a.x) || !Number.isFinite(a.y) || !Number.isFinite(a.radius)) {
        throw new Error(`${vp.name}: Arc ${k} contains non-finite values: ${JSON.stringify(a)}`);
      }
    }
    for (let i = 0; i < 5; i++) {
      const a0 = evalResult.actualCrosshairCenters[i * 2];
      const a1 = evalResult.actualCrosshairCenters[i * 2 + 1];
      if (Math.abs(a0.x - a1.x) > 1e-6 || Math.abs(a0.y - a1.y) > 1e-6) {
        throw new Error(`${vp.name}: Target ${i} crosshair pair centers do not coincide: (${a0.x}, ${a0.y}) vs (${a1.x}, ${a1.y})`);
      }
    }

    // 定义 5 个目标的独立第一性原理真实物理参数预期（全 6 维度 x, y, w, h, cx, cy）
    const targetsExpected = [
      // 目标 0: banana (正常像素目标)
      {
        name: 'Target 0 (banana)',
        expX: indepOffsetX + ((261.2 - 125.8 / 2) / srcW) * indepRenderW,
        expY: indepOffsetY + ((375.9 - 45.4 / 2) / srcH) * indepRenderH,
        expW: (125.8 / srcW) * indepRenderW,
        expH: (45.4 / srcH) * indepRenderH,
        expCx: indepOffsetX + (261.2 / srcW) * indepRenderW,
        expCy: indepOffsetY + (375.9 / srcH) * indepRenderH
      },
      // 目标 1: cut_left (像素目标，左侧出画贴边截断至 x0=0)
      {
        name: 'Target 1 (cut_left)',
        expX: indepOffsetX + 0.0 * indepRenderW,
        expY: indepOffsetY + ((360 - 100 / 2) / srcH) * indepRenderH,
        expW: ((10 + 40 / 2) / srcW) * indepRenderW,
        expH: (100 / srcH) * indepRenderH,
        expCx: indepOffsetX + (10 / srcW) * indepRenderW,
        expCy: indepOffsetY + (360 / srcH) * indepRenderH
      },
      // 目标 2: cut_right (归一化目标，右侧出画贴边截断至 x1=1)
      {
        name: 'Target 2 (cut_right)',
        expX: indepOffsetX + (0.95 - 0.4 / 2) * indepRenderW,
        expY: indepOffsetY + (0.5 - 0.2 / 2) * indepRenderH,
        expW: (1.0 - (0.95 - 0.4 / 2)) * indepRenderW,
        expH: 0.2 * indepRenderH,
        expCx: indepOffsetX + 0.95 * indepRenderW,
        expCy: indepOffsetY + 0.5 * indepRenderH
      },
      // 目标 3: cut_top (像素目标，顶侧出画贴边截断至 y0=0)
      {
        name: 'Target 3 (cut_top)',
        expX: indepOffsetX + ((640 - 100 / 2) / srcW) * indepRenderW,
        expY: indepOffsetY + 0.0 * indepRenderH,
        expW: (100 / srcW) * indepRenderW,
        expH: ((15 + 50 / 2) / srcH) * indepRenderH,
        expCx: indepOffsetX + (640 / srcW) * indepRenderW,
        expCy: indepOffsetY + (15 / srcH) * indepRenderH
      },
      // 目标 4: cut_bottom (归一化目标，底侧出画贴边截断至 y1=1)
      {
        name: 'Target 4 (cut_bottom)',
        expX: indepOffsetX + (0.5 - 0.2 / 2) * indepRenderW,
        expY: indepOffsetY + (0.95 - 0.3 / 2) * indepRenderH,
        expW: 0.2 * indepRenderW,
        expH: (1.0 - (0.95 - 0.3 / 2)) * indepRenderH,
        expCx: indepOffsetX + 0.5 * indepRenderW,
        expCy: indepOffsetY + 0.95 * indepRenderH
      }
    ];

    for (let i = 0; i < targetsExpected.length; i++) {
      const exp = targetsExpected[i];
      const actualBox = evalResult.actualStrokeRects[i];
      const actualArc = evalResult.actualCrosshairCenters[i * 2];

      // 验证预期值的有限性
      if (!Number.isFinite(exp.expX) || !Number.isFinite(exp.expY) || !Number.isFinite(exp.expW) || !Number.isFinite(exp.expH) || !Number.isFinite(exp.expCx) || !Number.isFinite(exp.expCy)) {
        throw new Error(`${vp.name} ${exp.name}: Expected values contain non-finite numbers`);
      }

      const errX = Math.abs(actualBox.x - exp.expX);
      const errY = Math.abs(actualBox.y - exp.expY);
      const errW = Math.abs(actualBox.w - exp.expW);
      const errH = Math.abs(actualBox.h - exp.expH);
      const errCx = Math.abs(actualArc.x - exp.expCx);
      const errCy = Math.abs(actualArc.y - exp.expCy);

      // 验证误差值本身的有限性
      if (!Number.isFinite(errX) || !Number.isFinite(errY) || !Number.isFinite(errW) || !Number.isFinite(errH) || !Number.isFinite(errCx) || !Number.isFinite(errCy)) {
        throw new Error(`${vp.name} ${exp.name}: Measured error is non-finite (NaN or Infinity)`);
      }

      const targetMaxErr = Math.max(errX, errY, errW, errH, errCx, errCy);
      if (!Number.isFinite(targetMaxErr)) {
        throw new Error(`${vp.name} ${exp.name}: targetMaxErr is NaN`);
      }

      if (targetMaxErr > globalMaxError) globalMaxError = targetMaxErr;
      totalCheckedDimensions += 6;

      log(`  [${exp.name}]:`);
      log(`    Expected Box: x=${exp.expX.toFixed(4)}, y=${exp.expY.toFixed(4)}, w=${exp.expW.toFixed(4)}, h=${exp.expH.toFixed(4)}`);
      log(`    Actual   Box: x=${actualBox.x.toFixed(4)}, y=${actualBox.y.toFixed(4)}, w=${actualBox.w.toFixed(4)}, h=${actualBox.h.toFixed(4)}`);
      log(`    Expected Crosshair: cx=${exp.expCx.toFixed(4)}, cy=${exp.expCy.toFixed(4)}`);
      log(`    Actual   Crosshair: cx=${actualArc.x.toFixed(4)}, cy=${actualArc.y.toFixed(4)}`);
      log(`    Errors: dx=${errX.toFixed(6)}px, dy=${errY.toFixed(6)}px, dw=${errW.toFixed(6)}px, dh=${errH.toFixed(6)}px, dcx=${errCx.toFixed(6)}px, dcy=${errCy.toFixed(6)}px [Max: ${targetMaxErr.toFixed(6)}px]`);

      // 严格使用 >= 0.001 拒绝边界值与非有限值
      if (targetMaxErr >= 0.001) {
        throw new Error(`${vp.name} ${exp.name}: Dimension error exceeds strict tolerance: ${targetMaxErr.toFixed(6)}px >= 0.001px`);
      }
    }

    await page.waitForTimeout(100);
    const screenshotPath = path.join(artifactDir, `playwright_overlay_calibrated_${vp.name}.png`);
    await page.screenshot({ path: screenshotPath, fullPage: true });
    log(`  Screenshot saved: ${screenshotPath}`);

    await page.evaluate(() => clearInterval(window.__overlayKeepAlive));
    await page.close();
  }

  await browser.close();
  log('=====================================================================');
  log(`SUMMARY STATS: Total Checked Dimensions = ${totalCheckedDimensions}`);
  log(`GLOBAL MAXIMUM MEASURED ERROR = ${globalMaxError.toFixed(6)}px`);
  log('ALL ASSERTIONS SATISFIED STRICTLY (all values finite, error < 0.001px)!');
  log('=====================================================================');

  fs.writeFileSync(path.join(artifactDir, 'playwright_test.log'), logLines.join('\n') + '\n', 'utf8');
})();
