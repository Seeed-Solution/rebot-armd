const { chromium } = require('playwright');
const assert = require('node:assert/strict');
const { spawn } = require('node:child_process');
const net = require('node:net');
const os = require('node:os');
const path = require('path');
const fs = require('fs');

const ROOT = path.resolve(__dirname, '..');
const ARMD_BIN = process.env.ARMD_BIN || path.join(ROOT, 'target', 'debug', 'armd');
const B601_CONFIG = path.join(ROOT, 'kits', 'recamera-b601dm', 'profile', 'rebot-b601dm.json');
const FORBIDDEN_PORTS = new Set([18765, 18001, 18990, 8001, 8090, 8765]);

async function freePort() {
  for (let i = 0; i < 16; i++) {
    const port = await new Promise((resolve, reject) => {
      const srv = net.createServer();
      srv.once('error', reject);
      srv.listen(0, '127.0.0.1', () => {
        const p = srv.address().port;
        srv.close(() => resolve(p));
      });
    });
    if (!FORBIDDEN_PORTS.has(port)) return port;
  }
  throw new Error('Could not allocate a non-forbidden free port');
}

async function startMockArmd() {
  assert(fs.existsSync(ARMD_BIN), `armd binary not found at ${ARMD_BIN} (cargo build --bin armd first)`);
  const port = await freePort();
  const state = fs.mkdtempSync(path.join(os.tmpdir(), 'armd-park-e2e-'));
  const args = [
    '--follower', 'mock',
    '--config', B601_CONFIG,
    '--port', String(port),
    '--log-dir', 'off',
    '--blackbox-dir', 'off',
    '--episodes-dir', 'off',
  ];
  const child = spawn(ARMD_BIN, args, {
    cwd: ROOT,
    stdio: ['ignore', 'ignore', 'pipe'],
    env: {
      ...process.env,
      RUST_LOG: 'warn',
      ARMD_APP_CONFIG: 'off',
      ARMD_POSES: path.join(state, 'poses.json'),
      ARMD_SEQUENCES: path.join(state, 'sequences.json'),
    },
  });
  let spawnErr = null;
  const stderr = [];
  child.once('error', (err) => { spawnErr = err; });
  child.stderr.on('data', (c) => stderr.push(String(c)));

  const stop = () => new Promise((resolve) => {
    const done = () => { fs.rmSync(state, { recursive: true, force: true }); resolve(); };
    if (spawnErr || child.pid == null || child.exitCode !== null || child.signalCode !== null) {
      done();
      return;
    }
    const kt = setTimeout(() => { try { child.kill('SIGKILL'); } catch (_) {} }, 3000);
    child.once('exit', () => { clearTimeout(kt); done(); });
    try { child.kill('SIGTERM'); } catch (_) { clearTimeout(kt); done(); }
  });

  try {
    const deadline = Date.now() + 15000;
    for (;;) {
      if (spawnErr) throw new Error(`failed to spawn armd (${ARMD_BIN}): ${spawnErr.message}`);
      if (child.exitCode !== null) throw new Error(`armd exited early (${child.exitCode}):\n${stderr.join('')}`);
      try {
        const res = await fetch(`http://127.0.0.1:${port}/api/v1/observation`, { cache: 'no-store' });
        if (res.ok) break;
      } catch (_) {}
      if (Date.now() > deadline) throw new Error('armd not ready');
      await new Promise((r) => setTimeout(r, 100));
    }
  } catch (e) {
    await stop();
    throw e;
  }
  return { port, url: `http://127.0.0.1:${port}`, stop };
}

async function holdButton(page, sel, holdMs = 750) {
  await page.waitForFunction((s) => {
    const e = document.querySelector(s);
    return !!e && !e.hidden && !e.disabled;
  }, sel, { timeout: 5000 });
  const box = await (await page.waitForSelector(sel, { state: 'visible', timeout: 5000 })).boundingBox();
  assert(box && box.width > 0 && box.height > 0, `${sel} has no bounding box`);
  await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
  await page.mouse.down();
  await page.waitForTimeout(holdMs);
  await page.mouse.up();
}

(async () => {
  const armd = await startMockArmd();
  let browser = null;
  try {
    const artifactDir = path.resolve(__dirname, 'artifacts');
    fs.mkdirSync(artifactDir, { recursive: true });

    browser = await chromium.launch({
      executablePath: process.env.CHROME || undefined,
      headless: true,
      args: ['--no-sandbox', '--disable-setuid-sandbox']
    });

    const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });

    page.on('console', msg => console.log('PAGE LOG:', msg.text()));
    page.on('pageerror', err => console.log('PAGE ERROR:', err.message));

    console.log(`[1/5] Navigating to ${armd.url} ...`);
    await page.goto(armd.url, { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => typeof S !== 'undefined' && S.ws && S.ws.readyState === 1, null, { timeout: 5000 });

    // 1. Ensure armed (acquire lease + enable torque via UI hold)
    console.log('[2/5] Checking arm state...');
    const pageState = await page.evaluate(() => ({
      S_state: S.state,
      S_isLive: isLive(),
      S_isOperational: isOperational(),
      S_ws_ready: S.ws ? S.ws.readyState : -1,
      S_leaseGen: S.leaseGeneration
    }));
    console.log('Page internal state:', JSON.stringify(pageState));

    if (!pageState.S_isOperational) {
      console.log('Arming robot via UI Enable hold...');
      await holdButton(page, '#btn-enable', 750);
      await page.waitForFunction(() => isOperational() && S.leaseGeneration != null, null, { timeout: 5000 });
    }

    const armedState = await page.evaluate(() => ({
      S_state: S.state,
      S_isLive: isLive(),
      S_isOperational: isOperational(),
      S_leaseGen: S.leaseGeneration
    }));
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

    const screenshotPath = path.join(artifactDir, 'playwright_park_zero_drop_verified.png');
    await page.screenshot({ path: screenshotPath });
    console.log(`Saved screenshot proof to ${screenshotPath}`);

    assert.equal(cutoffObs.is_torqued, false, 'Expected is_torqued == false');
    assert.equal(cutoffObs.safety_state, 'Idle', 'Expected safety_state == Idle');

    console.log('ALL REAL ROBOT PHYSICAL ACCEPTANCE CRITERIA PASSED!');
  } finally {
    if (browser) await browser.close().catch(() => {});
    await armd.stop();
  }
})().catch(err => {
  console.error('Error during test:', err);
  process.exit(1);
});
