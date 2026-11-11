"""Reference values for tests/test_rs_profile.rs (B601-RS chain, src/chain.rs).

Independent of armd's code:
- FK of gripper_end from the MuJoCo model rs_arm.xml (body pos / quat, hinge axes), a
  separate description of the same arm.
- Gravity holding torques from ReBot_Arm_RS.urdf as the numerical derivative of the
  potential energy V(q) = sum m g z_com over all links below base_link (fingers at 0).

Run from the repository root:
  uv run --with numpy python tests/fixtures/gen_rs_reference.py \
      <reBot-DevArm>/Rebot_Arm_description/RS > tests/fixtures/rs_reference.json
"""
import json
import sys
import xml.etree.ElementTree as ET

import numpy as np

G = 9.81
JOINTS = [f"joint{i}" for i in range(1, 7)]
SAMPLES = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.3, 1.0, 1.2, -0.4, 0.6, 1.1],
    [-1.2, 2.0, 0.5, 1.0, -1.0, -2.5],
    [2.1, 0.4, 2.6, -1.3, 1.4, 0.2],
    [-0.6, 2.8, 2.9, 0.7, -0.3, 3.0],
    [1.5, 1.57, 0.1, 1.5, 1.5, -1.0],
]


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat(w, x, y, z):
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def axis_angle(a, t):
    a = np.asarray(a, float) / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * k + (1 - np.cos(t)) * k @ k


def vec(s, default="0 0 0"):
    return np.array([float(v) for v in (s or default).split()])


def mjcf_fk(path, q):
    root = ET.parse(path).getroot()
    body = root.find("worldbody").find("body")  # base_link
    R, p = np.eye(3), vec(body.get("pos"))
    qmap = dict(zip(JOINTS, q))
    while body.get("name") != "gripper_end":
        nxt = None
        for child in body.findall("body"):
            if child.get("name") in ("link1", "link2", "link3", "link4", "link5", "link6", "gripper_end"):
                nxt = child
        body = nxt
        p = p + R @ vec(body.get("pos"))
        if body.get("quat"):
            R = R @ quat(*vec(body.get("quat")))
        j = body.find("joint")
        if j is not None and j.get("name") in qmap:
            R = R @ axis_angle(vec(j.get("axis")), qmap[j.get("name")])
    return R, p


def urdf_energy(path, q):
    root = ET.parse(path).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    joints = root.findall("joint")
    qmap = dict(zip(JOINTS, q))
    frames = {"base_link": (np.eye(3), np.zeros(3))}
    todo = ["base_link"]
    while todo:
        parent = todo.pop()
        for j in joints:
            if j.find("parent").get("link") != parent:
                continue
            o = j.find("origin")
            Rj = rpy(*vec(o.get("rpy") if o is not None else None))
            pj = vec(o.get("xyz") if o is not None else None)
            R, p = frames[parent]
            Rc, pc = R @ Rj, p + R @ pj
            if j.get("name") in qmap:
                Rc = Rc @ axis_angle(vec(j.find("axis").get("xyz")), qmap[j.get("name")])
            child = j.find("child").get("link")
            frames[child] = (Rc, pc)
            todo.append(child)
    v = 0.0
    for name, (R, p) in frames.items():
        inertial = links[name].find("inertial")
        if inertial is None or name == "base_link":
            continue
        m = float(inertial.find("mass").get("value"))
        com = p + R @ vec(inertial.find("origin").get("xyz"))
        v += m * G * com[2]
    return v


def main():
    rs = sys.argv[1]
    mjcf, urdf = f"{rs}/mjcf/rs_arm.xml", f"{rs}/urdf/ReBot_Arm_RS.urdf"
    out = []
    for q in SAMPLES:
        R, p = mjcf_fk(mjcf, q)
        h = 1e-6
        tau = []
        for i in range(6):
            qp, qm = list(q), list(q)
            qp[i] += h
            qm[i] -= h
            tau.append((urdf_energy(urdf, qp) - urdf_energy(urdf, qm)) / (2 * h))
        out.append({"q": q, "p": p.tolist(), "r": R.tolist(), "gravity_nm": tau})
    json.dump({"source": "rs_arm.xml (FK), ReBot_Arm_RS.urdf (gravity, dV/dq)", "samples": out}, sys.stdout, indent=1)
    print()


if __name__ == "__main__":
    main()
