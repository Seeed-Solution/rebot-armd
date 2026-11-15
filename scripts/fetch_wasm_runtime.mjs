#!/usr/bin/env node
/**
 * Fetch browser MuJoCo WASM (mujoco-js@0.0.7) and ONNX Runtime Web (onnxruntime-web@1.21.0)
 * from registry.npmmirror.com into assets/wasm/ (never embedded in armd binary).
 *
 * Usage:
 *   node scripts/fetch_wasm_runtime.mjs [--dir assets/wasm] [--force]
 */

import { execFileSync } from "node:child_process";
import crypto from "node:crypto";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(HERE, "..");

const PACKAGES = [
  {
    name: "mujoco-js",
    version: "0.0.7",
    url: "https://registry.npmmirror.com/mujoco-js/-/mujoco-js-0.0.7.tgz",
    sha256: "ee7443760feb69a8871b90fbab6b362ec45a71f777d1fb447c479456d46722d4",
    files: ["package/dist/mujoco_wasm.js", "package/dist/mujoco_wasm.d.ts", "package/dist/mujoco_wasm.wasm"],
    required: ["mujoco_wasm.js"],
  },
  {
    name: "onnxruntime-web",
    version: "1.21.0",
    url: "https://registry.npmmirror.com/onnxruntime-web/-/onnxruntime-web-1.21.0.tgz",
    sha256: "9c6b3e0955aa8bc61b69f62873bb3bdb021010a6d1550b6a638a74af353f445c",
    files: [
      "package/dist/ort.wasm.min.js",
      "package/dist/ort-wasm-simd-threaded.mjs",
      "package/dist/ort-wasm-simd-threaded.wasm",
      "package/dist/ort-wasm-simd-threaded.jsep.mjs",
      "package/dist/ort-wasm-simd-threaded.jsep.wasm",
    ],
    required: ["ort.wasm.min.js", "ort-wasm-simd-threaded.wasm"],
  },
];

function parseArgs(argv) {
  let outDir = path.join(REPO_ROOT, "assets", "wasm");
  let force = false;
  for (let i = 0; i < argv.length; i += 1) {
    const a = argv[i];
    if (a === "--dir" && argv[i + 1]) {
      outDir = path.resolve(argv[++i]);
    } else if (a === "--force") {
      force = true;
    } else if (a === "--help" || a === "-h") {
      console.log("Usage: node scripts/fetch_wasm_runtime.mjs [--dir PATH] [--force]");
      process.exit(0);
    }
  }
  return { outDir, force };
}

function listTarMembers(tgzPath) {
  return execFileSync("tar", ["-tzf", tgzPath], { encoding: "utf8" })
    .split(/\r?\n/)
    .filter(Boolean);
}

function main() {
  const { outDir, force } = parseArgs(process.argv.slice(2));
  fs.mkdirSync(outDir, { recursive: true });
  const stageDir = fs.mkdtempSync(path.join(outDir, ".stage-"));

  try {
    const manifest = { fetched_at: new Date().toISOString(), packages: {} };
    for (const pkg of PACKAGES) {
      const needed = pkg.required.every((f) => fs.existsSync(path.join(outDir, f)));
      if (needed && !force) {
        console.log(`[skip] ${pkg.name}@${pkg.version} already present in ${outDir}`);
        continue;
      }

      const tgzPath = path.join(stageDir, `${pkg.name}-${pkg.version}.tgz`);
      console.log(`[download] ${pkg.url}`);
      execFileSync("curl", ["-fSL", "--retry", "3", "-o", tgzPath, pkg.url], {
        stdio: "inherit",
      });

      const tgzBuf = fs.readFileSync(tgzPath);
      const actualSha256 = crypto.createHash("sha256").update(tgzBuf).digest("hex");
      if (actualSha256 !== pkg.sha256) {
        throw new Error(
          `SHA-256 verification failed for ${pkg.name}@${pkg.version}: expected ${pkg.sha256}, got ${actualSha256}`,
        );
      }
      console.log(`[verify] ${pkg.name}@${pkg.version} sha256 OK`);

      const members = new Set(listTarMembers(tgzPath));
      const extractDir = path.join(stageDir, pkg.name);
      fs.mkdirSync(extractDir, { recursive: true });
      const distMembers = [...members].filter(
        (m) => m.startsWith("package/dist/") && !m.endsWith("/"),
      );
      if (distMembers.length === 0) {
        throw new Error(`No package/dist/* entries found in ${pkg.url}`);
      }
      execFileSync("tar", ["-xzf", tgzPath, "-C", extractDir, ...distMembers], {
        stdio: "inherit",
      });

      const copied = [];
      for (const entry of distMembers) {
        const src = path.join(extractDir, entry);
        const base = path.basename(entry);
        // Copy requested files, or all dist wasm/js/mjs/d.ts for mujoco-js so single-file vs split wasm both work.
        const want =
          pkg.files.includes(entry) ||
          (pkg.name === "mujoco-js" && /\.(js|mjs|wasm|ts)$/i.test(base));
        if (!want || !fs.existsSync(src)) continue;
        const dst = path.join(outDir, base);
        fs.copyFileSync(src, dst);
        copied.push({ file: base, bytes: fs.statSync(dst).size });
      }
      for (const reqFile of pkg.required) {
        if (!fs.existsSync(path.join(outDir, reqFile))) {
          throw new Error(`Required runtime file missing after extract: ${reqFile}`);
        }
      }
      manifest.packages[pkg.name] = { version: pkg.version, sha256: pkg.sha256, files: copied };
      console.log(
        `[ok] ${pkg.name}@${pkg.version} -> ${copied.map((c) => c.file).join(", ")}`,
      );
    }

    const manifestPath = path.join(outDir, "manifest.json");
    if (Object.keys(manifest.packages).length > 0 || !fs.existsSync(manifestPath)) {
      fs.writeFileSync(manifestPath, `${JSON.stringify(manifest, null, 2)}\n`);
    }
  } finally {
    fs.rmSync(stageDir, { recursive: true, force: true });
  }
}

main();
