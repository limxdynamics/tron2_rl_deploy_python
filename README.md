<!--
  SPDX-FileCopyrightText: 2024-2026 LimX Dynamics Technology Co., Ltd.
  SPDX-License-Identifier: Apache-2.0
-->

<p align="center">
  <img src="doc/banner.png" alt="tron2_rl_deploy_python: the sole-foot, wheel-foot and dual-arm TRON2A robots this repository deploys RL policies to" width="100%">
</p>

<p align="center">
  <b>English</b> | <a href="README_zh-CN.md">中文</a>
</p>

<p align="center">
  <a href="https://github.com/limxdynamics/tron2_rl_deploy_python/actions/workflows/ci.yml"><img src="https://github.com/limxdynamics/tron2_rl_deploy_python/actions/workflows/ci.yml/badge.svg?branch=master" alt="CI"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-Apache--2.0-2f6feb" alt="License: Apache-2.0"></a>
  <img src="https://img.shields.io/badge/Python-3-2f6feb" alt="Python 3">
  <img src="https://img.shields.io/badge/ONNX%20Runtime-CPU-2f6feb" alt="ONNX Runtime">
  <img src="https://img.shields.io/badge/ROS-not%20required-2f6feb" alt="ROS not required">
</p>

# tron2-rl-deploy-python

> **Distribution.** The primary public distribution point for this
> repository is GitHub:
> <https://github.com/limxdynamics/tron2_rl_deploy_python>. The internal
> LimX GitLab is a mirror; open issues, PRs, and security reports on
> the GitHub repository.

Reinforcement-learning **deployment / inference** stack (Python) for the TRON2A humanoid — Sole-Foot
(`SF_TRON2A`), Wheel-Foot (`WF_TRON2A`), and dual-arm Sole-Foot (`DASF_TRON2A`) variants. Loads ONNX
policies via `onnxruntime` and drives joint targets through the LimX low-level SDK, either against the
MuJoCo simulator or against a physical robot.

## Highlights

- **Three robots, one entry point.** `main.py` picks the SF, WF or DA-SF controller from `ROBOT_TYPE`.
- **Sim and robot from the same command.** No argument connects to a local MuJoCo simulator;
  `python3 main.py <robot-ip>` drives the robot.
- **Policies included.** Policy and history encoder ONNX files for all three variants ship in
  `controllers/model/`; see [`MODEL_CARD.md`](MODEL_CARD.md).
- **Gamepad direct or through the SDK.** A Logitech F710 is read through pygame; without it the
  controller falls back to SDK `SensorJoy`.
- **Dry run before the real thing.** `python3 main.py <robot-ip> --dry-run` (SF / WF) checks and
  prints joint commands without sending them.

## Supported robots

| | Model | Description | Joints / policy actions | Format |
|:-:|---|---|:-:|---|
| <img src="doc/model_sf.png" width="320"> | `SF_TRON2A` | Biped with sole feet | 10 / 10 | ONNX policy + encoder |
| <img src="doc/model_wf.png" width="320"> | `WF_TRON2A` | Biped with wheel feet | 10 / 10 | ONNX policy + encoder |
| <img src="doc/model_dasf.png" width="320"> | `DASF_TRON2A` | Sole-foot biped with two arms, driven through the Centaur SDK | 26 / 20 | ONNX policy + encoder |

## How it works

<p align="center">
  <img src="doc/sim2real_en.png" alt="The same policy.onnx drives DASF_TRON2A in MuJoCo with python3 main.py and on the real robot with python3 main.py and the robot IP" width="100%">
</p>

## Quick start

In simulation, with [tron2_mujoco_sim](https://github.com/limxdynamics/tron2_mujoco_sim) running
`ROBOT_TYPE=SF_TRON2A uv run simulator.py` in another terminal:

```bash
git clone --recurse-submodules https://github.com/limxdynamics/tron2_rl_deploy_python.git
cd tron2_rl_deploy_python
pip install numpy scipy pyyaml onnxruntime pygame
pip install limxsdk-lowlevel/python3/amd64/limxsdk-*.whl
export ROBOT_TYPE=SF_TRON2A
python3 main.py
```

On the gamepad, press `L1 + B` to stand, then `L1 + Y` to walk; the left stick moves the robot and
the right stick turns it.

## 1. Directory layout

- `main.py`: controller entry point (auto-selects the SF / WF / DA-SF controller based on `ROBOT_TYPE`).
- `controllers/SolefootController.py`: `SF_TRON2A` inference and control logic.
- `controllers/WheelfootController.py`: `WF_TRON2A` inference and control logic.
- `controllers/DASFController.py`: `DASF_TRON2A` dual-channel Centaur SDK inference and control logic.
- `controllers/model/<ROBOT_TYPE>/`: per-variant model and configuration directory.
- `limxsdk-lowlevel/`: LimX SDK sources and examples (`git submodule`).

## 2. Environment setup

Python 3.10+. The Quick start installs the CPU runtime set and the x86_64 SDK wheel; two things it
does not cover:

```bash
pip install -U pip                                            # if your pip is older than the wheel
pip install limxsdk-lowlevel/python3/aarch64/limxsdk-*.whl    # on aarch64
```

The SDK wheel has to match the interpreter that runs `main.py`. `limxsdk-lowlevel/` is a submodule: if
you cloned without `--recurse-submodules`, run `git submodule update --init --recursive` before
installing the wheel.

## 3. Model files

Six ONNX files ship in the repository — one policy and one history encoder per variant — next to each
variant's `params.yaml`:

```text
controllers/model/
├── SF_TRON2A/      policy.onnx  encoder.onnx  params.yaml
├── WF_TRON2A/      policy.onnx  encoder.onnx  params.yaml
└── DASF_TRON2A/    policy.onnx  encoder.onnx  params.yaml
```

`params.yaml` is the per-variant runtime configuration: joint order, `loop_frequency`, PD gains,
action scaling, and the `commands.max` velocity limits that the joystick axes are scaled by. For the
weights themselves see [`MODEL_CARD.md`](MODEL_CARD.md).

## 4. Running the controller

```bash
cd tron2_rl_deploy_python
export ROBOT_TYPE=SF_TRON2A     # or WF_TRON2A / DASF_TRON2A

python3 main.py                 # no argument: local simulator at 127.0.0.1

# NOTE: <robot-ip> is a placeholder token — substitute your own
# robot / simulator IP before running. Nothing in this repo's
# source hard-codes a private IP.
python3 main.py <robot-ip>
```

## 5. Working with the MuJoCo simulator

Make sure the simulator side and the controller side use the same `ROBOT_TYPE`:

- Simulator side: `simulator.py` in [tron2_mujoco_sim](https://github.com/limxdynamics/tron2_mujoco_sim)
- Controller side: `tron2-rl-deploy-python/main.py`

We recommend starting the simulator first, then the controller.

DA-SF uses the same entry point with `ROBOT_TYPE=DASF_TRON2A`. On DA-SF, lower and upper body motor
state, commands and IMU all travel over the Centaur SDK; gamepad input is read directly through pygame
and only falls back to the SDK `SensorJoy` channel when pygame or a gamepad is unavailable. A
non-loopback robot IP always enables the real-hardware joint transform (thigh yaw zero offsets and
reversed knee/ankle-pitch directions); a loopback MuJoCo target does not use that transform.

### Sim-only mode

Before ever running against a physical robot, verify the full control loop against the LimX MuJoCo
simulator:

1. Start the simulator (`simulator.py` in tron2_mujoco_sim) with the same `ROBOT_TYPE` you plan to use.
2. Leave the controller's target IP at its default `127.0.0.1` (this is what `main.py` uses when no
   argument is passed).
3. Run `python3 main.py` and confirm the observation stream, action outputs, and joystick binds all
   behave as expected in simulation.

Only after sim-only verification passes should you consider a real-hardware run — and only with the
robot suspended / mounted, per [`SECURITY.md`](SECURITY.md#real-hardware-safety-notice).

## 6. Gamepad control notes

- `L1 + B` (`○`): STAND, interpolate from the current pose to the default pose
- `L1 + Y` (`△`): WALK, start the policy
- `L1 + A` (`×`): IDLE, joints held with damping only
- `R1`: clear velocity command and policy history

Button indices follow the F710 in `X` mode (`A`=0, `B`=1, `X`=2, `Y`=3, `L1`=4, `R1`=5); see
`JOY_BTNS` in the controllers.

Connect the F710 in `X` mode and run `main.py`. `robot-joystick` is not needed when the controller
finds the pad itself, which it reports differently per variant:

- SF / WF: `pygame joystick ready: <name>`, plus `Pygame Joystick: ENABLED` in the startup banner.
- DA-SF: `Direct joystick ready: <name>`, plus `Joystick source: direct pygame/F710`.

If the controller reports `Joystick source: limxsdk SensorJoy` or
`pygame joystick: no device found, fallback to SDK joystick topic`, it is listening to the SDK joy
topic instead. Run the gamepad publisher in another terminal:

```
../tron2_mujoco_sim/robot-joystick/robot-joystick
```

Press `L1 + B` and wait for the default pose, then press `L1 + Y`.

## 7. Screenshots / GIFs

### MuJoCo simulation deployment

| SF_TRON2A | WF_TRON2A | DASF_TRON2A |
| :---: | :---: | :---: |
| <img src="doc/sfmj-ezgif.com-video-to-gif-converter.gif" alt="SF Simulation" width="260"> | <img src="doc/wfmj-ezgif.com-video-to-gif-converter.gif" alt="WF Simulation" width="260"> | <img src="doc/mujoco_dasf.gif" alt="DASF MuJoCo Simulation" width="260"> |

### Real-world deployment

Suspend the robot before starting the controller when deploying on real hardware.

<img src="doc/deploy.jpg" alt="Deploy" width="360">

| SF_TRON2A | WF_TRON2A | DASF_TRON2A |
| :---: | :---: | :---: |
| <img src="doc/sf.GIF" alt="SF Real-world" width="260"> | <img src="doc/wf.GIF" alt="WF Real-world" width="260"> | <img src="doc/dasf.GIF" alt="DASF Real-world" width="260"> |

## 8. FAQ

- `ROBOT_TYPE not set`: run `export ROBOT_TYPE=...` first.
- `Model not found`: check that all files are present under `controllers/model/<ROBOT_TYPE>/`.
- `No module named limxsdk`: the SDK wheel is not installed in the current Python environment.
- `RobotState has not been received yet`: usually means the simulator is not running, or the two sides
  have mismatched `ROBOT_TYPE`.

---

## Verification

Two checks you can run on a workstation with no robot attached, once the SDK wheel is installed:

```bash
# Modules import, without calling robot.init.
python -c "import controllers"

# Every ONNX graph loads (no inference).
python -c "import onnx, glob
for p in glob.glob('controllers/model/*/*.onnx'):
    onnx.checker.check_model(onnx.load(p)); print('OK', p)"
```

The PR-time set — byte-compile, `ruff`, the `params.yaml` check and the unresolved-licence grep — is in
[`CONTRIBUTING.md`](CONTRIBUTING.md#verification-before-opening-a-pr). Of those, CI runs `ruff`, the
byte-compile step and the licence grep on every push and pull request; nothing in CI needs a robot.

## Cite & support

If you use this deployment stack in academic or public work, please cite the repository:

```
@misc{limx_tron2_rl_deploy_python_2026,
  title  = {tron2-rl-deploy-python: TRON2 RL deployment (Python)},
  author = {LimX Dynamics},
  year   = {2026},
  howpublished = {\url{https://github.com/limxdynamics/tron2_rl_deploy_python}}
}
```

- **Bug reports, feature requests and questions:**
  [GitHub Issues](https://github.com/limxdynamics/tron2_rl_deploy_python/issues).
- **Security reports / robot-safety incidents:** email `contact@limxdynamics.com`; see
  [`SECURITY.md`](SECURITY.md).
- **Company / commercial contact:** <https://www.limxdynamics.com>.

## License & attribution

This project is licensed under the **Apache License, Version 2.0** (January 2004). See the
[`LICENSE`](LICENSE) file for the full text. SPDX identifier: `Apache-2.0`.

- [`NOTICE`](NOTICE) — required attribution notice.
- [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) — per-item provenance for the checked-in ONNX
  weights, the `limxsdk-lowlevel` submodule, Python runtime dependencies (`onnxruntime`, `numpy`,
  `scipy`, `pyyaml`, `pygame`), and documentation media.
- [`MODEL_CARD.md`](MODEL_CARD.md) — model cards for all six SF/WF/DASF ONNX files.
- [`SECURITY.md`](SECURITY.md) — how to report a vulnerability, plus the real-hardware safety notice.
- [`CONTRIBUTING.md`](CONTRIBUTING.md) — dev setup (`pip`, `ruff`), verification steps, DCO sign-off,
  and the model-provenance rule.
- [`CHANGELOG.md`](CHANGELOG.md) — release notes and pending owner sign-off items.

## Scope

**Included** in this repository:

- Python controller entry point (`main.py`) and per-variant controllers
  (`controllers/SolefootController.py`, `controllers/WheelfootController.py`, and
  `controllers/DASFController.py`).
- Runtime configuration (`controllers/model/*/params.yaml`).
- Six ONNX inference blobs (`policy.onnx`, `encoder.onnx` for `SF_TRON2A`, `WF_TRON2A`, and
  `DASF_TRON2A`) — **pending provenance sign-off**; see [`MODEL_CARD.md`](MODEL_CARD.md) and
  [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) §2.
- Vendor SDK **submodule reference** (`limxsdk-lowlevel/`) — **pending SDK owner / legal clearance**;
  do not move the pin without written sign-off.

**Not included** — by design:

- No PyTorch / training checkpoints (`.pt`, `.pth`, `.ckpt`, `.safetensors`).
- No SDK binaries (`.so`, `.dll`, `.dylib`, `.lib`, `.whl`) — install the LimX SDK wheel yourself from
  the vendored `limxsdk-lowlevel/` submodule.
- No firmware, no factory calibration values, no per-serial calibration files.
- No rosbag / MCAP / trajectory captures.
- No customer- or site-specific configuration.
- **No pre-baked robot IP.** Nothing in this repository's source hard-codes a private IP. The
  internal-only sibling `tron2-rl-deploy-ros` retains a documentation-example literal `10.192.1.2` in
  its source / launch files, declared in that repo's `SECURITY.md`; this repository intentionally does
  not.

> **Real-hardware notice.** This is **not a simulation-only demo**. `main.py` opens an SDK connection
> to whatever IP you pass, and the controller writes joint torques that a real robot will execute.
> DA-SF controls both body channels through the Centaur SDK. Read
> [`SECURITY.md`](SECURITY.md#real-hardware-safety-notice) before you point this code at a physical
> machine.
