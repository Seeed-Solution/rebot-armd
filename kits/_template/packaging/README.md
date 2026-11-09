# packaging/ (skeleton)

Packaging contract: one install that contains

| Item | Path in the skeleton | Notes |
|---|---|---|
| armd binary | `/usr/bin/armd` | from an armd release artifact; do not rebuild a patched core |
| arm profile | `/usr/share/my-kit/profile/my-arm.json` | passed with `--config` |
| sidecar processes (optional) | e.g. `/usr/bin/my-vision` | perception provider, started before armd |
| launch script | [`etc/init.d/S90my-kit`](etc/init.d/S90my-kit) | starts the sidecar and `armd --follower <model> --config <profile>` |

The skeleton init script is minimal (no pid identity check, no stop confirmation). For a
real arm, follow the official kit's script
(`kits/recamera-b601dm/packaging/rootfs/etc/init.d/K92rebot-teleop`): it refuses to start
next to an untracked armd and does not report a stop until armd has exited.

Environment read by armd: `PORT`, `RUST_LOG`, `ARMD_DETECTOR_URL`, `ARMD_GRASP_SETUP`.

The package format (deb, opkg, tarball, container) is the kit's choice. The official kit
builds a deb with `scripts/build_deb.sh` (`--flavor recamera`).
