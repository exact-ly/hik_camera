# hik_camera

Minimal Python wrapper for Hikrobot GigE cameras.

This repository is intentionally scoped to:

- capture frames as RGB `numpy.ndarray`
- set/get basic camera parameters: exposure and gain

Everything else (raw/DNG pipeline, automatic recovery, multi-camera helper layer, extra demos/tests/docs) has been removed.

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

## RGB-only behavior

- The camera is forced to `PixelFormat=RGB8Packed`.
- The library validates the first frame during `__enter__`.
- If output is not RGB `uint8` (`H x W x 3`), initialization fails with a clear error.
- Raw/Bayer handling is intentionally unsupported in this version.

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
