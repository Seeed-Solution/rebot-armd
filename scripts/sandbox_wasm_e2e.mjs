#!/usr/bin/env node
/**
 * Browser WASM Sandbox E2E Test Suite
 *
 * Verifies end-to-end execution of the browser WASM sandbox:
 * 1. Spawns armd follower mock on an ephemeral port with sandbox and wasm asset overrides.
 * 2. Generates and uploads a linear ONNX policy (w=eye(7), b=[0.05, 0, 0, 0, 0, 0, 0])
 *    and a multimodal ONNX policy (with image input).
 * 3. Activates pcb_suction scene, suction tool, and ONNX policy in wasm mode.
 * 4. Boots Chromium (headless with SwiftShader) at ?sim=wasm.
 * 5. Verifies policy execution, 3D rendering, multimodal inference, and a 3-stage
 *    PCB pick-and-place workflow with vacuum I/O, task progression, LED updates,
 *    and telemetry assertions.
 */

import { execFileSync, spawn } from 'node:child_process';
import fs from 'node:fs';
import net from 'node:net';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const pythonBin = '/home/harvest/project/rebot-companion/.venv/bin/python';
const armdBin = path.join(root, 'target/debug/armd');
const wasmDir = path.join(root, 'assets/wasm');
const assetDir = path.join(root, 'kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/assets');
const builtinScenesDir = '/home/harvest/project/rebot-companion/rebot_companion/scenes/builtin';

const playwrightPath = process.env.PLAYWRIGHT || '/home/harvest/project/chaihuo-mcv-site/node_modules/playwright/index.mjs';
const chromePath = process.env.CHROME || '/home/harvest/.cache/ms-playwright/chromium-1255/chrome-linux-arm64/chrome';

async function getFreePort() {
  return new Promise((resolve, reject) => {
    const srv = net.createServer();
    srv.listen(0, '127.0.0.1', () => {
      const port = srv.address().port;
      srv.close((err) => (err ? reject(err) : resolve(port)));
    });
    srv.on('error', reject);
  });
}

async function waitForHttpOk(url, timeoutMs = 15000) {
  const deadline = Date.now() + timeoutMs;
  let lastErr = null;
  while (Date.now() < deadline) {
    try {
      const r = await fetch(url);
      if (r.ok) return true;
    } catch (e) {
      lastErr = e;
    }
    await new Promise((r) => setTimeout(r, 150));
  }
  throw new Error(`Timeout waiting for ${url}: ${lastErr ? lastErr.message : 'HTTP not OK'}`);
}

async function main() {
  console.log('--- Starting Browser WASM Sandbox E2E Test Suite ---');
  const port = await getFreePort();
  const tmpSandboxDir = fs.mkdtempSync(path.join(os.tmpdir(), 'armd-sandbox-'));
  const tmpPolicyDir = fs.mkdtempSync(path.join(os.tmpdir(), 'armd-policy-'));

  console.log(`Ephemeral port: ${port}`);
  console.log(`Sandbox dir: ${tmpSandboxDir}`);
  console.log(`Policy dir: ${tmpPolicyDir}`);

  let armdProc = null;
  let browser = null;
  const pageErrors = [];
  const consoleErrors = [];

  const diagnostics = {
    steps_completed: [],
    telemetry: null,
    test_pass_led_rgb: null,
    linear_policy_steps: 0,
    multimodal_policy_steps: 0,
    positions_j0: null,
  };

  try {
    // -------------------------------------------------------------------------
    // Step 1: Generate ONNX Policies (linear + multimodal)
    // -------------------------------------------------------------------------
    console.log('\n[1/6] Generating test ONNX policies in python...');
    const pyScript = `
import pathlib, sys, numpy as np
from rebot_companion.onnx_policy import scaffold_onnx_policy, build_linear_onnx_bytes

pol_dir = pathlib.Path(sys.argv[1])
p1 = scaffold_onnx_policy(pol_dir / "e2e_pol", "e2e_pol", action_dim=7)
w = np.eye(7, dtype=np.float32)
b = np.array([0.05, 0, 0, 0, 0, 0, 0], dtype=np.float32)
(p1 / "model.onnx").write_bytes(build_linear_onnx_bytes("observation.state", "action", w, b))

p2 = scaffold_onnx_policy(pol_dir / "e2e_multi_pol", "e2e_multi_pol", action_dim=7, include_image=True)
print("Policy packages generated successfully")
`;
    execFileSync(pythonBin, ['-c', pyScript, tmpPolicyDir], { stdio: 'inherit' });
    diagnostics.steps_completed.push('policies_generated');

    // -------------------------------------------------------------------------
    // Step 2: Spawn armd
    // -------------------------------------------------------------------------
    console.log('\n[2/6] Spawning armd on port ' + port + '...');
    const armdEnv = {
      ...process.env,
      ARMD_SANDBOX_DIR: tmpSandboxDir,
      ARMD_WASM_DIR: wasmDir,
      ARMD_ASSET_DIR: assetDir,
      ARMD_BUILTIN_SCENES_DIR: builtinScenesDir,
    };

    armdProc = spawn(
      armdBin,
      ['--follower', 'mock', '--port', String(port), '--log-dir', 'off', '--blackbox-dir', 'off', '--episodes-dir', 'off'],
      { env: armdEnv, stdio: ['ignore', 'pipe', 'pipe'] }
    );

    armdProc.stdout.on('data', (d) => {
      const s = d.toString().trim();
      if (s) console.log(`[armd:stdout] ${s}`);
    });
    armdProc.stderr.on('data', (d) => {
      const s = d.toString().trim();
      if (s) console.log(`[armd:stderr] ${s}`);
    });

    const catalogUrl = `http://127.0.0.1:${port}/api/v1/sandbox/catalog`;
    await waitForHttpOk(catalogUrl, 15000);
    console.log('armd catalog responded HTTP 200');
    diagnostics.steps_completed.push('armd_ready');

    // -------------------------------------------------------------------------
    // Step 3: Upload policies and activate wasm mode
    // -------------------------------------------------------------------------
    console.log('\n[3/6] Uploading policies and activating pcb_suction scene...');
    const uploadPolicy = async (id, dir) => {
      const pJson = JSON.parse(fs.readFileSync(path.join(dir, 'policy.json'), 'utf8'));
      const mB64 = fs.readFileSync(path.join(dir, 'model.onnx')).toString('base64');
      const res = await fetch(`http://127.0.0.1:${port}/api/v1/sandbox/policies`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          id,
          policy_json: pJson,
          model_onnx_base64: mB64,
        }),
      });
      if (!res.ok) {
        const text = await res.text();
        throw new Error(`Upload policy '${id}' failed (${res.status}): ${text}`);
      }
      return res.json();
    };

    await uploadPolicy('e2e_pol', path.join(tmpPolicyDir, 'e2e_pol'));
    await uploadPolicy('e2e_multi_pol', path.join(tmpPolicyDir, 'e2e_multi_pol'));
    console.log('Policies e2e_pol and e2e_multi_pol uploaded');

    const activateRes = await fetch(`http://127.0.0.1:${port}/api/v1/sandbox/activate`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        mode: 'wasm',
        scene: 'pcb_suction',
        tool: 'suction',
        policy: 'e2e_pol',
        auto_start_policy: true,
      }),
    });
    if (!activateRes.ok) {
      const text = await activateRes.text();
      throw new Error(`Activate failed (${activateRes.status}): ${text}`);
    }
    console.log('Scene pcb_suction activated in wasm mode with e2e_pol');
    diagnostics.steps_completed.push('sandbox_activated');

    // -------------------------------------------------------------------------
    // Step 4: Launch Headless Chromium & Navigate
    // -------------------------------------------------------------------------
    console.log('\n[4/6] Launching Chromium and opening cockpit (?sim=wasm)...');
    const { chromium } = await import(playwrightPath);
    browser = await chromium.launch({
      headless: true,
      executablePath: chromePath,
      args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader'],
    });

    const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
    page.on('pageerror', (err) => {
      const msg = err.message || String(err);
      console.error(`[pageerror] ${msg}`);
      pageErrors.push(msg);
    });
    page.on('console', (msg) => {
      if (msg.type() === 'error') {
        const txt = msg.text();
        const locUrl = msg.location()?.url || '';
        // Ignore expected non-fatal standalone responses (502 detection proxy, 8001 ws, favicon.ico 404)
        if (/Failed to load resource: the server responded with a status of 502|^WebSocket connection to .*(\/detection\/ws|:8001)/.test(txt)) return;
        if (locUrl.endsWith('/favicon.ico') && /status of 404/.test(txt)) return;
        console.error(`[console.error] ${txt} (${locUrl})`);
        consoleErrors.push(`${txt} (${locUrl})`);
      }
    });

    const pageUrl = `http://127.0.0.1:${port}/?sim=wasm`;
    await page.goto(pageUrl, { waitUntil: 'domcontentloaded' });
    console.log(`Navigated to ${pageUrl}`);

    // Wait for WASM_SIM ready and at least 3 policy steps
    console.log('Waiting for window.WASM_SIM.ready === true && policySteps >= 3...');
    await page.waitForFunction(
      () => window.WASM_SIM?.ready === true && (window.WASM_SIM?.policySteps ?? 0) >= 3,
      null,
      { timeout: 20000 }
    );

    const simState = await page.evaluate(() => window.WASM_SIM.getState());
    diagnostics.linear_policy_steps = simState.policy?.steps ?? 0;
    diagnostics.positions_j0 = simState.positions[0];
    console.log(`Linear policy executed ${diagnostics.linear_policy_steps} steps, positions[0] = ${diagnostics.positions_j0}`);

    if (simState.positions[0] <= 0.01) {
      throw new Error(`Expected positions[0] > 0.01 after policy steps, got ${simState.positions[0]}`);
    }

    const view3dVisible = await page.evaluate(() => {
      const el = document.getElementById('view3d');
      if (!el) return false;
      const cs = getComputedStyle(el);
      return !el.hidden && cs.display !== 'none' && cs.visibility !== 'hidden';
    });
    if (!view3dVisible) {
      throw new Error('#view3d element is not visible');
    }

    // Wait for V3.sandboxGroup to be rendered
    await page.waitForFunction(
      () => (window.V3?.sandboxGroup?.children?.length ?? 0) >= 10,
      null,
      { timeout: 10000 }
    );
    const sandboxChildrenCount = await page.evaluate(() => window.V3.sandboxGroup.children.length);
    console.log(`V3.sandboxGroup active with ${sandboxChildrenCount} meshes (>= 10 expected)`);
    diagnostics.steps_completed.push('linear_policy_and_scene_verified');

    // -------------------------------------------------------------------------
    // Step 5: Verify multimodal policy (include_image=True)
    // -------------------------------------------------------------------------
    console.log('\n[5/6] Verifying multimodal ONNX policy (with image input)...');
    const switchPolRes = await fetch(`http://127.0.0.1:${port}/api/v1/sandbox/activate`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        mode: 'wasm',
        scene: 'pcb_suction',
        tool: 'suction',
        policy: 'e2e_multi_pol',
        auto_start_policy: true,
      }),
    });
    if (!switchPolRes.ok) {
      throw new Error(`Switch to multimodal policy failed: ${switchPolRes.status}`);
    }

    await page.waitForFunction(
      () => window.WASM_SIM?.policyName === 'e2e_multi_pol' && (window.WASM_SIM?.policySteps ?? 0) >= 1,
      null,
      { timeout: 15000 }
    );
    diagnostics.multimodal_policy_steps = await page.evaluate(() => window.WASM_SIM.policySteps);
    console.log(`Multimodal policy verified: ${diagnostics.multimodal_policy_steps} steps executed with image input`);
    diagnostics.steps_completed.push('multimodal_policy_verified');

    // -------------------------------------------------------------------------
    // Step 6: 3-Stage PCB Suction Pick-and-Place Execution
    // -------------------------------------------------------------------------
    console.log('\n[6/6] Executing 3-stage PCB suction pick-and-place workflow...');
    // Disable policy override so moveTcp has exclusive motion control
    await page.evaluate(() => window.WASM_SIM.setPolicyEnabled(false));

    const moveTcpAndSettle = async (x, y, z, pitch = Math.PI / 2, label = '') => {
      const q7 = await page.evaluate(
        ({ x, y, z, pitch }) => window.WASM_SIM.moveTcp(x, y, z, pitch),
        { x, y, z, pitch }
      );
      if (!q7) throw new Error(`IK failed for moveTcp(${x}, ${y}, ${z}) [${label}]`);

      await page.waitForFunction(
        () => {
          const sim = window.WASM_SIM;
          if (!sim || !sim.overrideTargets || !sim.positions) return false;
          for (let i = 0; i < 4; i++) {
            if (Math.abs(sim.positions[i] - sim.overrideTargets[i]) > 0.04) return false;
          }
          return true;
        },
        null,
        { timeout: 5000 }
      );
      await page.waitForTimeout(150);
    };

    const postIo = async (ioMap) => {
      const res = await fetch(`http://127.0.0.1:${port}/api/v1/sandbox/io`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(ioMap),
      });
      if (!res.ok) throw new Error(`POST /api/v1/sandbox/io failed: ${res.status}`);
    };

    // Stage 1: Pick PCB from infeed
    console.log('-> Moving above infeed [0.23, -0.14, 0.11]...');
    await moveTcpAndSettle(0.23, -0.14, 0.11, Math.PI / 2, 'infeed_above');

    console.log('-> Descending onto PCB in infeed [0.23, -0.14, 0.016]...');
    await moveTcpAndSettle(0.23, -0.14, 0.016, Math.PI / 2, 'infeed_contact');

    console.log('-> Engaging vacuum suction...');
    await postIo({ vacuum: true });
    await page.waitForFunction(
      () => window.WASM_SIM?.io?.vacuum === true && window.WASM_SIM?.attachedObject === 'pcb',
      null,
      { timeout: 4000 }
    );
    await page.waitForTimeout(200);

    console.log('-> Lifting PCB from infeed [0.23, -0.14, 0.11]...');
    await moveTcpAndSettle(0.23, -0.14, 0.11, Math.PI / 2, 'infeed_lift');
    await page.waitForFunction(
      () => window.WASM_SIM?.getState()?.task?.stages?.pcb_picked === true,
      null,
      { timeout: 4000 }
    );
    console.log('   Task stage: pcb_picked = true');

    // Transfer over divider wall to test fixture
    console.log('-> Clearing divider wall [0.25, -0.065, 0.11]...');
    await moveTcpAndSettle(0.25, -0.065, 0.11, Math.PI / 2, 'divider_clear');

    console.log('-> Approaching test fixture [0.27, 0.01, 0.11]...');
    await moveTcpAndSettle(0.27, 0.01, 0.11, Math.PI / 2, 'fixture_approach');

    console.log('-> Descending onto test fixture pad [0.27, 0.01, 0.024]...');
    await moveTcpAndSettle(0.27, 0.01, 0.024, Math.PI / 2, 'fixture_place');

    console.log('-> Releasing vacuum suction on fixture...');
    await postIo({ vacuum: false });
    await page.waitForFunction(
      () => window.WASM_SIM?.io?.vacuum === false && !window.WASM_SIM?.attachedObject,
      null,
      { timeout: 4000 }
    );
    await page.waitForTimeout(300);

    // Stage 2: Verify pcb_in_fixture and LED mesh turns green
    await page.waitForFunction(
      () => window.WASM_SIM?.getState()?.task?.stages?.pcb_in_fixture === true,
      null,
      { timeout: 4000 }
    );
    console.log('   Task stage: pcb_in_fixture = true');

    await page.waitForFunction(
      () => {
        const mesh = window.V3?.sandboxGroup?.children?.find((c) => c.name === 'wasm:test_pass_led');
        if (!mesh?.material?.color) return false;
        return mesh.material.color.g > mesh.material.color.r;
      },
      null,
      { timeout: 4000 }
    );
    const ledColor = await page.evaluate(() => {
      const mesh = window.V3?.sandboxGroup?.children?.find((c) => c.name === 'wasm:test_pass_led');
      return mesh?.material?.color ? { r: mesh.material.color.r, g: mesh.material.color.g, b: mesh.material.color.b } : null;
    });
    diagnostics.test_pass_led_rgb = ledColor;
    console.log(`   test_pass_led mesh updated: r=${ledColor?.r.toFixed(3)}, g=${ledColor?.g.toFixed(3)}, b=${ledColor?.b.toFixed(3)} (green active)`);

    // Stage 3: Pick from fixture and transfer to outfeed
    console.log('-> Re-engaging vacuum suction on fixture...');
    await postIo({ vacuum: true });
    await page.waitForFunction(
      () => window.WASM_SIM?.io?.vacuum === true && window.WASM_SIM?.attachedObject === 'pcb',
      null,
      { timeout: 4000 }
    );
    await page.waitForTimeout(200);

    console.log('-> Lifting from fixture [0.27, 0.01, 0.10]...');
    await moveTcpAndSettle(0.27, 0.01, 0.10, Math.PI / 2, 'fixture_lift');

    console.log('-> Moving to outfeed bin [0.23, 0.15, 0.10]...');
    await moveTcpAndSettle(0.23, 0.15, 0.10, Math.PI / 2, 'outfeed_above');

    console.log('-> Descending into outfeed bin [0.23, 0.15, 0.030]...');
    await moveTcpAndSettle(0.23, 0.15, 0.030, Math.PI / 2, 'outfeed_descend');

    console.log('-> Releasing vacuum suction into outfeed bin...');
    await postIo({ vacuum: false });
    await page.waitForFunction(
      () => window.WASM_SIM?.io?.vacuum === false && !window.WASM_SIM?.attachedObject,
      null,
      { timeout: 4000 }
    );
    await page.waitForTimeout(400);

    // Trigger immediate telemetry push to armd
    await page.evaluate(() => window.WASM_SIM.pushTelemetry());

    // Poll armd backend state until telemetry marks task completed
    console.log('Waiting for backend state telemetry completion...');
    let finalBackendState = null;
    const deadline = Date.now() + 6000;
    while (Date.now() < deadline) {
      const r = await fetch(`http://127.0.0.1:${port}/api/v1/sandbox/state`);
      if (r.ok) {
        finalBackendState = await r.json();
        if (finalBackendState?.telemetry?.task?.completed) break;
      }
      await new Promise((r) => setTimeout(r, 200));
    }

    diagnostics.telemetry = finalBackendState?.telemetry;
    const task = finalBackendState?.telemetry?.task;

    if (!task) throw new Error('Backend state did not contain telemetry.task');
    if (task.stages?.pcb_picked !== true) throw new Error(`pcb_picked is not true: ${JSON.stringify(task.stages)}`);
    if (task.stages?.pcb_in_fixture !== true) throw new Error(`pcb_in_fixture is not true: ${JSON.stringify(task.stages)}`);
    if (task.stages?.pcb_in_outfeed !== true) throw new Error(`pcb_in_outfeed is not true: ${JSON.stringify(task.stages)}`);
    if (task.completed !== true) throw new Error(`task.completed is not true: ${JSON.stringify(task)}`);

    console.log('All 3 task stages verified completed in armd telemetry:');
    console.log(`  pcb_picked: ${task.stages.pcb_picked}`);
    console.log(`  pcb_in_fixture: ${task.stages.pcb_in_fixture}`);
    console.log(`  pcb_in_outfeed: ${task.stages.pcb_in_outfeed}`);
    console.log(`  task.completed: ${task.completed}`);
    diagnostics.steps_completed.push('pick_and_place_stages_completed');

    if (pageErrors.length > 0) {
      throw new Error(`Encountered page errors: ${JSON.stringify(pageErrors)}`);
    }
    if (consoleErrors.length > 0) {
      throw new Error(`Encountered unexpected console errors: ${JSON.stringify(consoleErrors)}`);
    }

    const summary = {
      status: 'PASS',
      diagnostics,
      pageErrorsCount: pageErrors.length,
      consoleErrorsCount: consoleErrors.length,
    };
    console.log('\n=== TEST RESULT: PASS ===');
    console.log(JSON.stringify(summary, null, 2));
    process.exitCode = 0;
  } catch (err) {
    console.error('\n=== TEST RESULT: FAIL ===');
    console.error(`Error: ${err.message || String(err)}`);
    console.error(`Page errors (${pageErrors.length}):`, pageErrors);
    console.error(`Console errors (${consoleErrors.length}):`, consoleErrors);
    console.error('Diagnostics:', JSON.stringify(diagnostics, null, 2));
    process.exitCode = 1;
  } finally {
    if (browser) {
      try { await browser.close(); } catch (_) {}
    }
    if (armdProc) {
      try {
        if (armdProc.exitCode === null) {
          armdProc.kill('SIGTERM');
          await new Promise((resolve) => {
            armdProc.once('exit', resolve);
            setTimeout(() => {
              try { armdProc.kill('SIGKILL'); } catch (_) {}
              resolve();
            }, 2000);
          });
        }
        if (armdProc.stdout) armdProc.stdout.destroy();
        if (armdProc.stderr) armdProc.stderr.destroy();
        armdProc.unref();
      } catch (_) {}
    }
    try { fs.rmSync(tmpSandboxDir, { recursive: true, force: true }); } catch (_) {}
    try { fs.rmSync(tmpPolicyDir, { recursive: true, force: true }); } catch (_) {}
  }
}

main();
