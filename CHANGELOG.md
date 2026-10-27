# Changelog

All notable changes to `tron2-rl-deploy-python` will be documented
here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Open-source scaffolding: `NOTICE`, `THIRD_PARTY_NOTICES.md`,
  `MODEL_CARD.md`, `SECURITY.md`, `CONTRIBUTING.md`, `CHANGELOG.md`.
- GitHub CI workflow: `ruff` lint, `python -m py_compile` over every
  `.py`, submodule status check, EXIF sanity over `doc/`, private-IP
  scan with allowlist, ONNX-provenance gate (fails if a new `*.onnx`
  is added without a matching `THIRD_PARTY_NOTICES.md` row and
  `MODEL_CARD.md` section), and a hard deny-list for `*.pt`, `*.pth`,
  `*.ckpt`, `*.so`, `*.dll`, `*.dylib`, `*.lib`, `*.whl`, `*.bag`,
  `*.mcap`.
- Issue templates and PR template under `.github/`, plus `CODEOWNERS`
  routing to maintainers / legal / model / sdk / robotics / safety
  teams.
- `MODEL_CARD.md` at the repo root with template rows for all six
  checked-in ONNX files (`SF_TRON2A/policy.onnx`,
  `SF_TRON2A/encoder.onnx`, `WF_TRON2A/policy.onnx`,
  `WF_TRON2A/encoder.onnx`, `DASF_TRON2A/policy.onnx`, and
  `DASF_TRON2A/encoder.onnx`).
- `DASF_TRON2A` deployment through `DASFController.py`, including
  Centaur SDK lower/upper-body channels and local-MuJoCo versus
  real-hardware joint-coordinate handling.
- `README.md`: SPDX header; "License & attribution" cross-links;
  "Scope / not included" (calls out the ONNX policy / encoder files
  pending provenance, real-hardware control code, and the
  `limxsdk-lowlevel` submodule pending clearance); "Verification"
  section with `python -m py_compile`, dry-run import, ONNX / YAML
  sanity, and sim-only guidance; "Cite & support" with
  `contact@limxdynamics.com`.

### Changed
- CI now runs on `master`, the default branch (it was set to `main` and
  had never run). Pre-existing failures fixed so every job passes:
  `ruff` findings in the controllers and tools (import cleanup,
  `range(0, n)` -> `range(n)`, `ClassVar` on the joystick maps, a
  `ruff.toml` that keeps the `*Controller.py` module names); the hard-coded
  `10.192.1.4` default in `tools/joy_probe.py` (now `127.0.0.1`, like
  `main.py`); the docs scan matching its own command and the phrase
  "no proprietary"; and metadata in `doc/` media (Douyin/CapCut device and
  user IDs in the comments of `doc/dasf.GIF` and `doc/mujoco_dasf.gif`
  removed; `doc/deploy.jpg` converted from Display P3 to sRGB without the
  embedded profile). Frames and images are otherwise unchanged.
- Repository links moved from `limx-tron2/tron2-rl-deploy-python` (which
  does not resolve) to `limxdynamics/tron2_rl_deploy_python` in the
  README distribution note, CONTRIBUTING, CHANGELOG links and the issue
  template; the issue template's "General questions" link now points at
  the README FAQ because Discussions are disabled.
- `README.md` / `README_zh-CN.md`: landing page with banner, model cards,
  sim-to-real picture, highlights and quick start (new images
  `doc/banner.png`, `doc/sim2real_{en,zh}.png`, `doc/model_{sf,wf,dasf}.png`;
  the banner and sim-to-real photos are frames from `doc/sf.GIF`,
  `doc/wf.GIF`, `doc/dasf.GIF` and `doc/mujoco_dasf.gif`). Gamepad notes
  now match the controllers (`L1 + B` stand, `L1 + Y` walk, `L1 + A`
  idle); SF / WF real-world GIF captions corrected; "Cite & support"
  links point at this repository; License, Scope and Distribution text
  unchanged, License and Scope moved to the end.
- `limxsdk-lowlevel` submodule pinned to `61624b5` (limxsdk 4.1.2). The
  SDK wheels are now `cp38-abi3-manylinux_2_29` builds, so install
  commands in the docs use `limxsdk-*.whl` instead of
  `limxsdk-*-py3-none-any.whl`.
- `DASF_TRON2A` controller: PS-style gamepad mapping (`L1 + ○` prepare,
  `L1 + △` start, `L1 + ×` stop); powers up in damped IDLE instead of
  moving to the default pose automatically.
- `.gitignore` expanded from a single-line `__pycache__` to a
  Python-appropriate ignore list (venv, caches, editor / OS junk,
  build artifacts), plus a hard deny-list for weight / SDK / bag
  artifacts. The six grandfathered ONNX files under
  `controllers/model/**` are exempted via an explicit `!` rule so the
  deny-list applies everywhere else in the tree.
- `README.md`, `SECURITY.md`, `CONTRIBUTING.md`, and command examples
  in shipped Markdown / YAML now consistently use `<robot-ip>` as a
  placeholder token that users must substitute with their own robot
  or simulator IP. No production or internal-network IP is embedded
  in this repository.

### Resolved (2026-07-16)
- **Private-IP handling** — resolved 2026-07-16 per owner decision.
  All Markdown / YAML command examples now use `<robot-ip>` as a
  placeholder token. No production IP is embedded in this
  repository. The sibling `tron2-rl-deploy-ros` retains a
  documentation-example literal `10.192.1.2` in its source / launch
  files, declared in that repo's `SECURITY.md`.

### Pending owner sign-off (blocks first public tag)
- **ONNX provenance for `controllers/model/SF_TRON2A/policy.onnx`** —
  training run, training data source and license boundary,
  redistribution terms. See
  [`MODEL_CARD.md`](MODEL_CARD.md#sf_tron2apolicyonnx).
- **ONNX provenance for `controllers/model/SF_TRON2A/encoder.onnx`** —
  same fields. See
  [`MODEL_CARD.md`](MODEL_CARD.md#sf_tron2aencoderonnx).
- **ONNX provenance for `controllers/model/WF_TRON2A/policy.onnx`** —
  same fields. See
  [`MODEL_CARD.md`](MODEL_CARD.md#wf_tron2apolicyonnx).
- **ONNX provenance for `controllers/model/WF_TRON2A/encoder.onnx`** —
  same fields. See
  [`MODEL_CARD.md`](MODEL_CARD.md#wf_tron2aencoderonnx).
- **ONNX provenance for `controllers/model/DASF_TRON2A/policy.onnx`** —
  same fields. See
  [`MODEL_CARD.md`](MODEL_CARD.md#dasf_tron2apolicyonnx).
- **ONNX provenance for `controllers/model/DASF_TRON2A/encoder.onnx`** —
  same fields. See
  [`MODEL_CARD.md`](MODEL_CARD.md#dasf_tron2aencoderonnx).
- **`limxsdk-lowlevel` submodule commit clearance** — SDK owner /
  legal must clear the upstream repository's license and confirm
  that consuming the SDK via a submodule pin is acceptable for a
  public release. Do **not** move the pin without written sign-off.
- **Model card completion** — every `⚠ TO CONFIRM` row in
  [`MODEL_CARD.md`](MODEL_CARD.md) resolved.
- **`controllers/model/*/params.yaml` review** — confirm no per-serial
  calibration constants are embedded.
- **Documentation media (`doc/*.jpg`, `doc/*.GIF`, `doc/*.gif`)** —
  EXIF strip and content review for individuals / facilities /
  non-public hardware.

## [0.1.0] — TBD

First public release. Contents:

- Python controller entry point and SF / WF / DASF controllers for
  `SF_TRON2A`, `WF_TRON2A`, and `DASF_TRON2A` variants.
- Six ONNX inference blobs under `controllers/model/*/` (subject to
  the Pending owner sign-off items above).
- LimX SDK consumed via the `limxsdk-lowlevel` submodule; SDK wheel is
  **not** vendored.

[Unreleased]: https://github.com/limxdynamics/tron2_rl_deploy_python/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/limxdynamics/tron2_rl_deploy_python/releases/tag/v0.1.0
