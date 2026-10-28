# Galaxy XR (SM-I610) Support

QuestLHSync support for the Samsung Galaxy XR headset, enabling SteamVR Lighthouse tracking alignment without SpaceCalibrator.

## Status

**Experimental** — the buffer discovery and readout pipeline is functional. Empirical confirmation with actual base stations is pending.

The S5K931 sensor is **NIR-optimized** with 60% quantum efficiency at 850nm ([Samsung Semiconductor](https://semiconductor.samsung.com/image-sensor/global-shutter-image-sensor/isocell-vizion-931/)), which is the exact wavelength used by SteamVR Lighthouse base stations. This makes it well-suited for detecting laser sweeps — likely better than the Quest Pro's OV7251, whose 850nm QE is lower (exact figure not publicly documented). An IR-cut filter on the tracking optics is unlikely: the ctrl-tracking cameras have an active IR torch (`torch-sequence` flash mode in QVR config), and all tracking cameras must see the controllers' IR LEDs for inside-out tracking to work.

## Requirements

- Samsung Galaxy XR (SM-I610) on firmware **I610UEU2AZF3** (July 2026)
  - The October 2026 update patches the DFRoot exploit (single TIPC kernel fix) — do not update
- [DFRoot](https://github.com/nickcbaker/DFRoot) 4.7+ for ephemeral root (CVE-2026-43284)
- [KernelSU-GXR](https://github.com/4uva4ek/KernelSU-GXR) v1.0.16+ (kernel module, not persistent)
- Root must be re-established after every reboot

## Architecture

### How it differs from Quest

| | Quest Pro / Quest 3 | Galaxy XR |
|---|---|---|
| **Frida target** | `vendor.oculus.hardware.sensors@1.0-service` | `android.hardware.perceptionhost` |
| **Tracking cameras** | OV7251, 640x480, Y8, global shutter | Samsung S5K931 (ISOCELL Vizion 931), 640x640, Y8, global shutter, 60% QE @ 850nm |
| **Calibration** | `/persist/calibration/camera_calibration.json` (JSON) | `/vendor/etc/qvr/device_calibration_default.xml` (QVR XML) |
| **Frame buffer size** | 307,200 bytes (640x480) | 409,600 bytes (640x640 single) / 819,200 bytes (1280x640 stereo) |
| **Tracking cameras** | 4 (2 front OG01A + 2 side OV7251) | 4 (2 tracking + 2 ctrl-tracking, all S5K931) |
| **Root method** | Magisk (persistent) | DFRoot + KernelSU (ephemeral, per-boot) |

### Camera layout

All tracking cameras use the Samsung S5K931 sensor (stereo pairs, global shutter):

| Camera | QVR IDs | Resolution | Purpose | Notes |
|--------|---------|------------|---------|-------|
| tracking A/B | 2, 3 | 1280x640 @60fps (640x640 per eye) | SLAM / 6DoF | Primary Lighthouse candidates |
| ctrl-tracking A/B | 6, 7 | 1280x640 @60fps (640x640 per eye) | Hand tracking (upper) | Has IR torch (`torch-sequence` flash mode) |
| face-tracking A/B | 4, 5 | 1280x640 @60fps | Face tracking | Not used for Lighthouse |
| eye-tracking L/R | 8-11 | 1280x640 @30fps | Eye tracking | Not used for Lighthouse |

### Process map

| Process | Role |
|---------|------|
| `android.hardware.perceptionhost` (PID ~1235) | Owns tracking camera dmabuf ring buffers — **Frida injection target** |
| `qvrservice` | Qualcomm VR service (manages camera config, minimal dmabufs) |
| `perceptionflinger` | Perception compositor |
| `vendor.qti.camera.provider@2.7-service_64` | Qualcomm camera HAL (RGB passthrough frames) |

### dmabuf buffer layout

When the headset is worn (cameras active), `perceptionhost` holds:

| Buffer size | Count | Interpretation |
|-------------|-------|----------------|
| 819,200 (1280x640 Y8) | ~80 | Stereo tracking frame pairs, double-mapped (2 mappings per inode) |
| 409,600 (640x640 Y8) | ~170 | Individual camera frames (eye/face/split stereo) |
| 32,768 | ~160 | Metadata / timestamps |

When the headset is **not worn**, the buffers exist but are frozen (proximity sensor suspends cameras).

## Building

```
python magisk/build_module.py galaxy_xr
```

This produces `out/QuestLHSync-galaxy-xr-module-<version>.zip`, a KernelSU module.

## Calibration

The Galaxy XR uses Qualcomm's QVR calibration system:

- **Runtime**: `/data/vendor/qvr/device_calibration.xml` (symlink, requires root)
- **Factory default**: `/vendor/etc/qvr/device_calibration_default.xml` (readable)

The XML contains per-camera intrinsics (FISHEYE_4_PARAMETERS distortion model), extrinsics (rig translation/rotation relative to IMU), and capture details. The backend wraps this in a `{"kind": "galaxy_xr", ...}` JSON envelope for the PC-side driver.

## Open questions

1. **End-to-end IR visibility**: The S5K931 has excellent 850nm sensitivity (60% QE) and an IR-cut filter is unlikely (the cameras must see controller IR LEDs, and ctrl-tracking has an active IR torch). A test with actual base stations will confirm the laser sweeps are detectable through the full optical path.
2. **Exposure cycle**: QuestLHSync relies on short-exposure frames for Lighthouse detection (ambient light filtered out). The Galaxy XR's exposure cycling pattern needs characterization.
3. **Camera identification**: The buffer discovery groups dmabufs by size, but we need to confirm which groups correspond to tracking vs. ctrl-tracking vs. face cameras. Mean brightness during active use distinguishes them (tracking ~44, ctrl-tracking with IR torch ~85-94).
4. **PC-side calibration parsing**: The SteamVR driver needs a parser for the `galaxy_xr` calibration kind (QVR XML with FISHEYE_4_PARAMETERS).

## Testing

A standalone test script (`lhtest2.js`) can be injected via Frida to check whether base station IR sweeps are visible:

```bash
# On PC with frida-tools installed, headset connected via ADB and rooted:
python run_lhtest2.py
# Runs 120 seconds, logs to /data/local/tmp/lhtest2.log on headset
# Toggle base station on/off during the test
```
