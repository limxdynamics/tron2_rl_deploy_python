# Contributing to `tron2-rl-deploy-python`

Thanks for helping improve the TRON2 RL deployment stack. This
repository is a **real-hardware controller**: the Python code here
runs a reinforcement-learning policy that drives joint torques on a
physical robot. The guidelines below aim to keep contributions safe,
reproducible, and legally clean.

## Table of contents

- [Ways to contribute](#ways-to-contribute)
- [Development setup](#development-setup)
- [Repository layout](#repository-layout)
- [Model files and provenance](#model-files-and-provenance)
- [Coding style](#coding-style)
- [Verification before opening a PR](#verification-before-opening-a-pr)
- [Commit messages](#commit-messages)
- [Pull request checklist](#pull-request-checklist)
- [Sign-off (DCO)](#sign-off-dco)
- [Code of conduct](#code-of-conduct)

## Ways to contribute

- Bug fixes in the SF / WF / DASF controllers (observation, action,
  safety clamps, IO plumbing).
- Simulator-side improvements (dry-run mode, sim-only launcher).
- Documentation, verification snippets, joystick / hardware notes.
- CI / lint / packaging.

We do **not** accept:

- New ONNX / PyTorch / checkpoint weights without a model card and
  written owner sign-off — see
  [Model files and provenance](#model-files-and-provenance).
- Vendor SDK binaries (`.so`, `.dll`, `.dylib`, `.whl`) checked into
  this repository. The LimX SDK is installed by the user from a
  vendor wheel; do not bundle it here.
- Hard-coded credentials, API keys, tokens, or non-example private
  IPs / hostnames.
- Factory calibration values, firmware, rosbags / MCAP, or
  customer-specific configuration.

## Development setup

Prerequisites:

- Python 3.10+ (matches the interpreter used to build the LimX SDK
  wheel you install).
- `pip`, `virtualenv` or `venv`.
- LimX SDK wheel from the `limxsdk-lowlevel` submodule.
- For local lint: `ruff` (recommended) or `pyflakes`.

```bash
git clone --recurse-submodules \
  https://github.com/limx-tron2/tron2-rl-deploy-python.git
cd tron2-rl-deploy-python

python -m venv .venv
source .venv/bin/activate

pip install -U pip
pip install numpy scipy pyyaml onnxruntime pygame
pip install ruff  # for lint

# LimX SDK wheel (pick your arch)
pip install limxsdk-lowlevel/python3/amd64/limxsdk-*.whl
# or
pip install limxsdk-lowlevel/python3/aarch64/limxsdk-*.whl
```

If you are only editing docs / CI / non-import-time code, you can skip
the SDK wheel install and rely on `python -m compileall` for basic
syntax checks.

## Repository layout

```
tron2-rl-deploy-python/
├── main.py                       # entry point (selects SF/WF/DASF by ROBOT_TYPE)
├── controllers/
│   ├── __init__.py
│   ├── SolefootController.py     # SF_TRON2A inference + control
│   ├── WheelfootController.py    # WF_TRON2A inference + control
│   ├── DASFController.py         # DASF_TRON2A inference + control
│   └── model/
│       ├── SF_TRON2A/            # policy.onnx, encoder.onnx, params.yaml
│       ├── WF_TRON2A/            # policy.onnx, encoder.onnx, params.yaml
│       └── DASF_TRON2A/          # policy.onnx, encoder.onnx, params.yaml
├── limxsdk-lowlevel/             # git submodule (vendor SDK, pinned)
├── doc/                          # README media (deploy.jpg, GIFs)
├── LICENSE, NOTICE, THIRD_PARTY_NOTICES.md, MODEL_CARD.md,
│   SECURITY.md, CONTRIBUTING.md, CHANGELOG.md, README.md
└── .github/                      # CI + issue / PR templates
```

## Model files and provenance

The six checked-in ONNX files under `controllers/model/*/` are the
single largest legal risk in this repository (see the review report
and [`MODEL_CARD.md`](MODEL_CARD.md)).

**Rules for model changes:**

- **Do not add** a new `*.onnx`, `*.pt`, `*.pth`, or `*.ckpt` file
  without:
  1. Adding a corresponding row in
     [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) §2.
  2. Adding / updating the model card entry in
     [`MODEL_CARD.md`](MODEL_CARD.md) with checkpoint id, training
     run, training data description, evaluation notes, intended use,
     out-of-scope use, and redistribution status.
  3. Written sign-off from the model owner and legal on training data
     boundary and redistribution.
- CI will fail any PR that adds a `*.onnx` without a matching model
  card row.
- **Do not silently remove** an existing ONNX file — removal is a
  legal / policy decision handled through the owner sign-off process,
  not through a code PR.
- Weight formats other than ONNX (`.pt`, `.pth`, `.ckpt`, `.safetensors`)
  are on the deny-list and rejected by CI.

## Coding style

- Follow PEP 8 (100-column soft limit).
- Prefer explicit imports; do not add wildcard imports.
- Use `ruff` for lint:

  ```bash
  ruff check .
  ```

- Keep control-loop code allocation-free where practical (pre-allocate
  numpy arrays in `__init__`, avoid creating tensors inside the tick).
- Never hard-code an IP address, hostname, credential, or file path
  outside the repository root. `<robot-ip>` in the shipped README /
  YAML examples is a **placeholder token** meant to be substituted
  by the user with their own robot or simulator IP; do not replace
  it with a real address, and do not introduce new hard-coded
  private IPs elsewhere in the tree.

## Verification before opening a PR

Run all of the following and paste the summary into the PR
description:

```bash
# 1. Byte-compile every Python file (catches syntax errors)
python -m compileall -q main.py controllers/

# 2. Lint
ruff check .

# 3. Dry-run import (no hardware required; requires the SDK wheel)
#    This exercises module-level side effects without calling robot.init.
python -c "import controllers"

# 4. YAML sanity
python -c "import yaml, glob
for p in glob.glob('controllers/model/*/params.yaml'):
    yaml.safe_load(open(p))
    print('OK', p)"

# 5. ONNX sanity (loads the graph; does not run inference)
python -c "import onnx, glob
for p in glob.glob('controllers/model/*/*.onnx'):
    onnx.checker.check_model(onnx.load(p)); print('OK', p)"

# 6. No unresolved TODO / proprietary / confidential markers in docs
grep -rniE 'proprietary|confidential|todo: license|unknown license' \
   README.md THIRD_PARTY_NOTICES.md MODEL_CARD.md \
   CHANGELOG.md CONTRIBUTING.md SECURITY.md
```

If your change affects the control tick, additionally test against the
LimX MuJoCo simulator (`127.0.0.1`) **before** touching a real robot.
Do not test motion changes for the first time on physical hardware.

## Commit messages

Follow Conventional Commits:

```
type(scope): short imperative summary

Longer explanation if needed.

Signed-off-by: Your Name <you@example.com>
```

`type` ∈ `feat | fix | docs | refactor | chore | ci | test | model`.
`scope` is usually the module (`solefoot`, `wheelfoot`, `main`,
`ci`, `docs`) or `meta` for repo-wide changes.

## Pull request checklist

- [ ] `python -m compileall` succeeds on every changed `.py`.
- [ ] `ruff check .` is clean (or diffed to explicit `noqa` with
      justification).
- [ ] `python -c "import controllers"` succeeds locally.
- [ ] No new `*.onnx` / `*.pt` / `*.pth` / `*.ckpt` / `*.so` / `*.dll` /
      `*.dylib` / `*.whl` added, **or** the addition includes a
      model-card row and legal sign-off.
- [ ] No hard-coded credentials, API keys, tokens, or private IPs /
      hostnames. The shipped `<robot-ip>` token is a placeholder for
      users to substitute, not a real address.
- [ ] `THIRD_PARTY_NOTICES.md` and, if models are touched,
      `MODEL_CARD.md` are updated.
- [ ] `CHANGELOG.md` has an entry under **[Unreleased]**.
- [ ] DCO sign-off on every commit.

## Sign-off (DCO)

We use the [Developer Certificate of Origin](https://developercertificate.org/).
Every commit must be signed off:

```bash
git commit -s -m "your message"
```

Signing off certifies that you have the right to submit the change
under the repository's license.

## Code of conduct

Be respectful and constructive. Reports to
`contact@limxdynamics.com`.
