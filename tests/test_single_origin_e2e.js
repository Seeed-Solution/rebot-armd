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
  const state = fs.mkdtempSync(path.join(os.tmpdir(), 'armd-single-origin-'));
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
  return { port, url: `http://127.0.0.1:${port}/`, stop };
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
    browser = await chromium.launch({
      executablePath: process.env.CHROME || undefined,
      headless: true,
      args: ['--no-sandbox', '--disable-setuid-sandbox']
    });

    const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });

    const consoleErrors = [];
    page.on('console', msg => {
      if (msg.type() === 'error') consoleErrors.push(msg.text());
    });

    console.log(`Navigating to Single-Origin Gateway: ${armd.url} ...`);
    await page.goto(armd.url, { waitUntil: 'networkidle', timeout: 15000 });

    // Wait for WebSocket telemetry connection (read-only: no lease yet, so no heartbeats)
    await page.waitForFunction(() => typeof S !== 'undefined' && S.ws && S.ws.readyState === 1 && S.epoch != null, null, { timeout: 5000 });

    const preLease = await page.evaluate(() => ({
      leaseGeneration: S.leaseGeneration,
      hbAt: S.hb ? S.hb.at : null,
    }));
    assert.equal(preLease.leaseGeneration, null, 'Expected no lease on initial read-only connect');
    assert.equal(preLease.hbAt, 0, 'Expected no heartbeats before lease is acquired');

    // Acquire lease via Enable hold button, then wait for heartbeats
    await holdButton(page, '#btn-enable', 750);
    await page.waitForFunction(
      () => typeof S !== 'undefined' && S.leaseGeneration != null && S.hb && S.hb.at > 0 && (S.hb.n > 0 || S.hb.hz > 0),
      null,
      { timeout: 5000 }
    );

    const state = await page.evaluate(() => ({
      wsReady: typeof S !== 'undefined' && S.ws ? S.ws.readyState : 0,
      wsCount: typeof S !== 'undefined' ? S.wsCount : 0,
      hz: typeof S !== 'undefined' ? S.hz : 0,
      leaseGeneration: typeof S !== 'undefined' ? S.leaseGeneration : null,
      hbCount: typeof S !== 'undefined' && S.hb ? S.hb.n : 0,
      hbHz: typeof S !== 'undefined' && S.hb ? S.hb.hz : 0,
      hbAt: typeof S !== 'undefined' && S.hb ? S.hb.at : 0,
      locationHost: location.host,
      videoMode: typeof V !== 'undefined' ? V.mode : null,
      videoWs: typeof CFG !== 'undefined' && CFG.video ? CFG.video.ws : null,
      dof: typeof S !== 'undefined' ? S.dof : null,
      title: document.title,
    }));

    console.log('Client State Extracted:', JSON.stringify(state, null, 2));

    const shotDir = path.resolve(__dirname, 'artifacts');
    fs.mkdirSync(shotDir, { recursive: true });
    const shotPath = path.join(shotDir, 'playwright_single_origin_gateway_verified.png');
    await page.screenshot({ path: shotPath, fullPage: true });
    console.log('Screenshot saved to:', shotPath);

    assert.equal(state.wsReady, 1, `WebSocket telemetry not connected (readyState != 1) on port ${armd.port}`);
    assert.notEqual(state.leaseGeneration, null, 'Expected active leaseGeneration');
    assert(state.hbAt > 0 && (state.hbCount > 0 || state.hbHz > 0), 'Expected heartbeats sent after lease acquired');
    assert.equal(state.locationHost, `127.0.0.1:${armd.port}`, `locationHost mismatch: ${state.locationHost}`);
    assert(state.videoWs, 'Expected CFG.video.ws to be defined');
    assert.equal(new URL(state.videoWs).hostname, '127.0.0.1', `videoWs hostname mismatch: ${state.videoWs}`);

    console.log('SUCCESS: All single-origin gateway end-to-end assertions passed!');
  } finally {
    if (browser) await browser.close().catch(() => {});
    await armd.stop();
  }
})().catch(err => {
  console.error('Error during test:', err);
  process.exit(1);
});
