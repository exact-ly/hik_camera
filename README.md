# hik_camera

Minimal Python wrapper for Hikrobot GigE cameras.

This repository is intentionally scoped to:

- capture frames as RGB `numpy.ndarray`
- set/get basic camera parameters: exposure and gain

Everything else (DNG pipeline, automatic recovery, multi-camera helper layer, extra demos/tests/docs) has been removed.

## Requirements

1. Install Hikrobot MVS SDK (Linux default path: `/opt/MVS`, Windows default path: `C:\Program Files (x86)\MVS`)
2. Python 3.8+
3. `uv` for environment and dependency management

If your SDK is installed in a non-default path, set `MVCAM_SDK_PATH`.

## Install (uv)

```bash
uv sync
```

## Usage

```python
from hik_camera import HikCamera

with HikCamera(ip="10.101.68.102") as cam:
    cam.set_exposure(50000)  # microseconds
    cam.set_gain(0.0)        # dB
    rgb = cam.get_frame()    # np.uint8, shape: (H, W, 3)
    print(rgb.shape, rgb.dtype)

    # compatibility aliases for existing wrappers
    cam["ExposureAuto"] = "Off"
    cam["GainAuto"] = "Off"
    cam["ExposureTime"] = 50000
    cam["Gain"] = 0.0
    rgb = cam.robust_get_frame()
```

## Capture Format

- `get_frame()` always returns RGB `uint8` (`H x W x 3`).
- By default, the library does not change the camera `PixelFormat`.
- Pass `capture_format` to request a specific camera output format.
- Supported capture formats include `RGB8Packed`, `BGR8Packed`, `Mono8`, `Mono16`, Bayer 8-bit, Bayer 12-bit packed, and unpacked Bayer 10/12/16-bit.

```python
from hik_camera import HikCamera

with HikCamera(ip="10.101.68.102", capture_format="RGB8Packed") as cam:
    rgb = cam.get_frame()

with HikCamera(ip="10.101.68.102", capture_format="BayerGB12Packed") as cam:
    rgb = cam.get_frame()  # Bayer is demosaiced internally, still RGB uint8 HxWx3
```

You can also set `PixelFormat` through `setting_items`; this is treated as explicit user configuration:

```python
with HikCamera(
    ip="10.101.68.102",
    setting_items=[("PixelFormat", "BayerGB12Packed")],
) as cam:
    rgb = cam.get_frame()
```

## Existing wrapper compatibility

- `HikCamera(..., setting_items=[("Width", 1920), ("Height", 1080), ...])` is supported.
- `camera["ParamName"] = value` is supported for common bool/int/float/string nodes.
- `robust_get_frame()` is available as a compatibility alias to `get_frame()`.

## Docker (uv-based)

```bash
docker build -t hik_camera .
docker run --net=host -e HIK_CAMERA_IP=10.101.68.102 -it hik_camera
```

The container runs:

```bash
uv run python -m hik_camera.hik_camera
```

Set `HIK_CAMERA_IP` to capture one frame in the demo entrypoint.
