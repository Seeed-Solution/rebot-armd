# Kit: reCamera + reBot B601-DM (official reference kit)

Controller: Seeed reCamera (SG2002) with the B4 CAN board. Arm: reBot B601-DM (6 joints and a
gripper, Damiao motors). Camera: the reCamera itself, on the gripper (eye-in-hand). Mount:
the printed holder in [`hardware/`](hardware/).

The kit adds data and packaging to the armd core; it does not change armd's code. How core
and kits fit together: [docs/architecture.md](../../docs/architecture.md).

| Directory | Contents | Contract |
|---|---|---|
| [`profile/`](profile/) | `rebot-b601dm.json`, the arm profile (equal to the built-in `ArmConfig::rebot_dm()`) | arm profile, `profile_version` 1 |
| [`packaging/`](packaging/) | `rebot-teleop` deb sources: init script, gallery manifest `rebot-teleop.json`, URDF, control files | packaging, deb 1.1.0 |
| [`vision/`](vision/) | placeholder for the kit's perception process (phase 2) | perception provider API |
| [`hardware/`](hardware/) | camera mount CAD v5 and v7_tilt (CERN-OHL-W-2.0) | - |
| [`calibration/`](calibration/) | camera tilt per mount, placeholder intrinsics, grasp setup defaults | - |
| [`docs/`](docs/README.md) | wiring, assembly, verified capabilities | - |

Build (from the repository root):

```bash
export RECAMERA_TOOLCHAIN=<path>/host-tools/gcc/riscv64-linux-musl-x86_64
./scripts/build_deb.sh                 # --flavor recamera (default): core + this kit
# -> dist/rebot-teleop_1.1.0_riscv64.deb
```

The deb installs the profile at `/usr/share/rebot-teleop/profile/rebot-b601dm.json`; the init
script passes it with `--config` when the follower is `rebot-dm` and the file exists.
