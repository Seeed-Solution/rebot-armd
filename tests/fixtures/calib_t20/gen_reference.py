# Usage: uv run --no-project --with opencv-contrib-python-headless --with numpy python gen_reference.py <dir with cal_obs.txt, f0*.jpg, hover_h*_2.jpg, hover_h*_obs.json> > ref.json
# fixtures.json = the sticker points / tip and the hover markers (rebot-vision corner order = cv2 order reversed) of ref.json.
# Ground truth for the T20 JS self-tests: sticker tip_cam (quad.py logic) and hover board corners (tipboard.py).
import cv2, sys, json, numpy as np
F = sys.argv[1]
cal = json.loads(open(F + '/cal_obs.txt').read().split('\n')[0])['calibration']; K = cal['intrinsics']
A = np.array([[K['fx'], 0, K['cx']], [0, K['fy'], K['cy']], [0, 0, 1]]); dist = np.array([K['k1'], K['k2'], 0, 0, K['k3']])
X0, Y0, X1, Y1 = 225, 415, 440, 500
out = {'intrinsics': {k: K[k] for k in ['fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'k3']}, 'sticker': {}, 'hover': {}}
pts_all = []
for f in ['f03', 'f04', 'f06']:
    g = cv2.imread(f'{F}/{f}.jpg', 0)[Y0:Y1, X0:X1].astype(np.float32)
    t, m = cv2.threshold(g.astype(np.uint8), 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    white = np.percentile(g, 90); b = []
    for i in range(1, n):
        x_, y_, w_, h_, ar = st[i]
        if ar < 60 or w_ / h_ > 3.5 or x_ == 0 or x_ + w_ >= X1 - X0: continue
        pad = 5; sx, sy = max(x_ - pad, 0), max(y_ - pad, 0)
        sub = np.clip(white - g[sy:y_ + h_ + pad, sx:x_ + w_ + pad], 0, None) * cv2.dilate((lab[sy:y_ + h_ + pad, sx:x_ + w_ + pad] == i).astype(np.uint8), np.ones((2 * pad + 1,) * 2, np.uint8))
        yy, xx = np.mgrid[0:sub.shape[0], 0:sub.shape[1]]; s = sub.sum()
        b.append((int(ar), (xx * sub).sum() / s + sx + X0, (yy * sub).sum() / s + sy + Y0 - 140, w_, h_))
    b = sorted(b, reverse=True)[:4]
    b = sorted(b, key=lambda t: t[2]); top = sorted(b[:2], key=lambda t: t[1]); bot = sorted(b[2:], key=lambda t: t[1])
    pts = [list(map(float, p[1:3])) for p in top + bot]
    pts_all.append(pts)
    out['sticker'][f] = {'otsu': float(t), 'white': float(white), 'points': pts, 'areas': [p[0] for p in top + bot]}
P = np.array(pts_all); img = P.mean(0)
for L1, L2 in [(10, 16), (16, 10)]:
    obj = np.array([[-4.4, L1, 0], [4.4, L1, 0], [-4.4, L2, 0], [4.4, L2, 0]]) / 1000
    ok, rv, tv, err = cv2.solvePnPGeneric(obj, img.reshape(-1, 1, 2), A, dist, flags=cv2.SOLVEPNP_IPPE)
    sols = [{'tip_cam_mm': (t.ravel() * 1000).tolist(), 'reproj_px': float(e[0])} for t, e in zip(tv, err)]
    out['sticker'][f'ippe_{L1}_{L2}'] = sols
tips = []
obj = np.array([[-4.4, 10, 0], [4.4, 10, 0], [-4.4, 16, 0], [4.4, 16, 0]]) / 1000
for p in P:
    ok, rv, tv, err = cv2.solvePnPGeneric(obj, p.reshape(-1, 1, 2), A, dist, flags=cv2.SOLVEPNP_IPPE)
    tips.append((tv[int(np.argmin([e[0] for e in err]))].ravel() * 1000).tolist())
out['sticker']['per_frame_tip_mm'] = tips
L, gp = 0.025, 0.005; p = L + gp
def corners(i):
    r, c = divmod(i, 4); cx = (c - 1.5) * p; cy = (r - 1) * p; h = L / 2
    return np.array([[cx - h, cy + h, 0], [cx + h, cy + h, 0], [cx + h, cy - h, 0], [cx - h, cy - h, 0]])
tip_cam = np.array([-0.10, 10.58, 95.41]) / 1000
d = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters())
for h in ['h1', 'h2', 'h4', 'h5', 'h6']:
    g_ = cv2.imread(f'{F}/hover_{h}_2.jpg', 0)[140:500]; cs, ids, _ = d.detectMarkers(g_)
    obs = json.load(open(f'{F}/hover_{h}_obs.json'))
    ob = []; im = []; mk = []
    for c, i in zip(cs, ids.ravel()):
        if i > 11: continue
        oc = corners(i); ob += [oc[3], oc[2], oc[1], oc[0]]; im += list(c.reshape(4, 2))
        mk.append({'id': int(i), 'corners_cv2': c.reshape(4, 2).tolist()})
    ob = np.array(ob, float); im = np.array(im, float)
    flags = cv2.SOLVEPNP_IPPE if len(ob) == 4 else cv2.SOLVEPNP_ITERATIVE
    ok, rv, tv = cv2.solvePnP(ob, im, A, dist, flags=flags); R, _ = cv2.Rodrigues(rv)
    tb = R.T @ (tip_cam - tv.ravel())
    out['hover'][h] = {'markers': mk, 'positions': obs['positions'], 'tcp': obs.get('tcp'), 'tip_board_mm': (tb * 1000).tolist(), 'height_mm': float(-tb[2] * 1000)}
print(json.dumps(out, indent=1))
