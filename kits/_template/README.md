# Kit template

A kit adapts armd to one combination of controller, arm, camera and mount. armd itself (the
core) stays unchanged; a kit provides data and packaging that use the four contracts in
[docs/architecture.md](../../docs/architecture.md):

| Contract | What the kit provides | Version |
|---|---|---|
| Arm profile | [`profile/`](profile/): an `ArmConfig` JSON for `armd --config` | `profile_version` 1 |
| Perception provider API | optional: a process serving the detector API that armd proxies ([`vision/`](vision/README.md)) | unversioned, as in docs/detection_api.md |
| Capability discovery | nothing: armd derives `capabilities` in `GET /api/v1/observation/schema` from the profile and driver | `capabilities.version` 4 |
| Packaging | [`packaging/`](packaging/README.md): armd binary + profile + optional sidecars + launch script | layout below |

Third-party kits live in their own repositories. They do not fork armd: they take the armd
binary from a release artifact (`armd-lite_<version>_riscv64.tar.gz` from
`./scripts/build_deb.sh --flavor lite`, or a build of the core for another target) and state
which contract versions they were written against. This directory is a copy-and-edit start.

## Steps

1. Copy `kits/_template/` into your repository and rename `my-kit`.
2. Write the profile: start from [`profile/example.json`](profile/example.json) (armd's generic
   6-joint profile; no kinematics, workspace, gripper, grasp or gravity compensation) or from
   the official [`kits/recamera-b601dm/profile/rebot-b601dm.json`](../recamera-b601dm/profile/rebot-b601dm.json).
   `dof` must match the follower model (`rebot-dm` 7; `rebot-rs`, `so-arm101`,
   `fashionstar`, `star-arm102` 6; `mock` any). Field list:
   [docs/control_safety_contract.md](../../docs/control_safety_contract.md#configuration--配置).
3. Check it with the mock follower: `armd --follower mock --config profile/my-arm.json`, then
   `curl http://127.0.0.1:8765/api/v1/observation/schema` and read `capabilities`.
4. If the kit has a camera process, serve the routes in [`vision/README.md`](vision/README.md)
   and point armd at it with `ARMD_DETECTOR_URL`.
5. Package: adapt [`packaging/`](packaging/README.md) to your controller's init system.
6. Document wiring, assembly and what you verified on hardware (see
   [`kits/recamera-b601dm/docs/`](../recamera-b601dm/docs/README.md) for the official kit's page).

## Official vs third party

Official kits are in this repository under `kits/` (today only `recamera-b601dm`), built
by `scripts/build_deb.sh` and covered by its tests. Everything else is third party: built and
supported by its authors, with the armd version and contract versions it targets stated in
its README. armd only supports the arm drivers it ships (`--follower`); a new motor protocol
is a change to the core, not a kit.
