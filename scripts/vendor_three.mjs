#!/usr/bin/env node
// Rebuilds src/api/vendor/three-urdf.min.js.gz (the cockpit's lazily loaded 3D view runtime) from pinned
// npm packages. Run in an empty directory:
//   npm install --save-exact three@0.186.1 urdf-loader@0.13.1 esbuild@0.25.10
//   node /path/to/armd/scripts/vendor_three.mjs
// Output: src/api/vendor/three-urdf.min.js.gz (gzip -9; armd embeds it and serves it with Content-Encoding: gzip),
// one ES module exporting only the three.js classes the view uses, OrbitControls, STLLoader and
// URDFLoader. urdf-loader's Collada support is replaced by a stub (the arm meshes are STL).
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createRequire } from 'node:module';
import zlib from 'node:zlib';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const req = createRequire(path.join(process.cwd(), 'package.json'));
const esbuild = req('esbuild');
import fs from 'node:fs';
const ver = (p) => JSON.parse(fs.readFileSync(path.join(process.cwd(), 'node_modules', p, 'package.json'), 'utf8')).version;
const pins = { three: '0.186.1', 'urdf-loader': '0.13.1' };
for (const [p, v] of Object.entries(pins)) if (ver(p) !== v) throw new Error(`${p} ${ver(p)} != pinned ${v}`);

const entry = `
export { Scene, PerspectiveCamera, WebGLRenderer, Color, HemisphereLight, DirectionalLight, GridHelper, Group,
  BoxGeometry, EdgesGeometry, LineSegments, LineBasicMaterial, Mesh, MeshStandardMaterial, MeshBasicMaterial,
  SphereGeometry, CylinderGeometry, BufferGeometry, Float32BufferAttribute, Line, Vector3, Quaternion, Box3,
  LoadingManager, SRGBColorSpace } from 'three';
export { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
export { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
export { default as URDFLoader } from 'urdf-loader/src/URDFLoader.js';
`;
const stubCollada = {
  name: 'stub-collada',
  setup(b) {
    b.onResolve({ filter: /ColladaLoader\.js$/ }, () => ({ path: 'collada-stub', namespace: 'stub' }));
    b.onLoad({ filter: /.*/, namespace: 'stub' }, () => ({ contents: 'export class ColladaLoader { constructor() { throw new Error("Collada not bundled"); } }' }));
  },
};
const out = path.join(root, 'src/api/vendor/three-urdf.min.js.gz');
const res = await esbuild.build({ write: false,
  stdin: { contents: entry, resolveDir: process.cwd(), loader: 'js' },
  bundle: true, format: 'esm', minify: true, legalComments: 'none', target: 'es2020', outfile: 'three-urdf.min.js', plugins: [stubCollada],
  banner: { js: `/* three.js ${pins.three} (MIT, Copyright 2010-2026 three.js authors) + urdf-loader ${pins['urdf-loader']} (Apache-2.0, Copyright 2020 California Institute of Technology); bundled by scripts/vendor_three.mjs. See THIRD_PARTY_NOTICES.md. */` },
});
const js = res.outputFiles[0].contents;
fs.writeFileSync(out, zlib.gzipSync(js, { level: 9 })); // gzip header mtime is 0: reproducible
console.log('wrote', out, js.length, 'bytes before gzip,', fs.statSync(out).size, 'after');
