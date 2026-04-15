#!/usr/bin/env python3

"""Minimal RGB-only wrapper for Hikrobot MVS camera SDK."""

from __future__ import annotations

import ctypes
from ctypes import byref, memset, sizeof
import os
import socket
import sys
from threading import Lock
from typing import Any, Iterable, Mapping

import numpy as np


if sys.platform.startswith("win"):
    MVCAM_SDK_PATH = os.environ.get("MVCAM_SDK_PATH", r"C:\Program Files (x86)\MVS")
    MV_IMPORT_DIR = os.path.join(MVCAM_SDK_PATH, r"Development\Samples\Python\MvImport")
else:
    MVCAM_SDK_PATH = os.environ.get("MVCAM_SDK_PATH", "/opt/MVS")
    MV_IMPORT_DIR = os.path.join(MVCAM_SDK_PATH, "Samples/64/Python/MvImport")


if MV_IMPORT_DIR not in sys.path:
    sys.path.insert(0, MV_IMPORT_DIR)

try:
    import MvCameraControl_class as hik
except ModuleNotFoundError as exc:
    raise ModuleNotFoundError(
        "Cannot import MvCameraControl_class. Install Hikrobot MVS SDK and verify "
        f"MVCAM_SDK_PATH or default SDK path: {MV_IMPORT_DIR}"
    ) from exc


def ip_to_int(ip: str) -> int:
    return sum(int(octet) << shift for octet, shift in zip(ip.split("."), [24, 16, 8, 0]))


def get_host_ip_by_target_ip(target_ip: str) -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect((target_ip, 80))
        return str(sock.getsockname()[0])


class HikCamera(hik.MvCamera):
    _FLOAT_PARAM_KEYS = {"ExposureTime", "Gain"}

    def __init__(
        self,
        ip: str,
        host_ip: str | None = None,
        timeout_ms: int = 40000,
        setting_items: Iterable[tuple[str, Any]] | Mapping[str, Any] | None = None,
    ) -> None:
        if not ip:
            raise ValueError("`ip` is required")

        super().__init__()
        self._ip = ip
        self.host_ip = host_ip or get_host_ip_by_target_ip(ip)
        self.timeout_ms = int(timeout_ms)

        self._lock = Lock()
        self._is_open = False
        self._payload_size = 0
        self._data_buf = None
        self._frame_info = None
        self._setting_items = self._normalize_setting_items(setting_items)

        self._create_handle()

    @staticmethod
    def _normalize_setting_items(
        setting_items: Iterable[tuple[str, Any]] | Mapping[str, Any] | None,
    ) -> tuple[tuple[str, Any], ...]:
        if setting_items is None:
            return ()
        if isinstance(setting_items, Mapping):
            items = list(setting_items.items())
        else:
            items = list(setting_items)
        return tuple((str(key), value) for key, value in items)

    @staticmethod
    def _check_ok(ret: int, action: str) -> None:
        if ret != 0:
            raise RuntimeError(f"{action} failed with code 0x{ret:08x}")

    @property
    def ip(self) -> str:
        return self._ip

    def _create_handle(self) -> None:
        st_dev_info = hik.MV_CC_DEVICE_INFO()
        st_gige = hik.MV_GIGE_DEVICE_INFO()
        st_gige.nCurrentIp = ip_to_int(self.ip)
        st_gige.nNetExport = ip_to_int(self.host_ip)
        st_dev_info.nTLayerType = hik.MV_GIGE_DEVICE
        st_dev_info.SpecialInfo.stGigEInfo = st_gige
        self._check_ok(self.MV_CC_CreateHandle(st_dev_info), "MV_CC_CreateHandle")

    def _set_enum(self, key: str, value: int) -> None:
        self._check_ok(
            self.MV_CC_SetEnumValue(key, value),
            f"MV_CC_SetEnumValue({key!r}, {value!r})",
        )

    def _set_enum_by_string(self, key: str, value: str) -> None:
        self._check_ok(
            self.MV_CC_SetEnumValueByString(key, value),
            f"MV_CC_SetEnumValueByString({key!r}, {value!r})",
        )

    def _set_bool(self, key: str, value: bool) -> None:
        self._check_ok(
            self.MV_CC_SetBoolValue(key, value),
            f"MV_CC_SetBoolValue({key!r}, {value!r})",
        )

    def _set_float(self, key: str, value: float) -> None:
        self._check_ok(
            self.MV_CC_SetFloatValue(key, float(value)),
            f"MV_CC_SetFloatValue({key!r}, {value!r})",
        )

    def _get_float(self, key: str) -> float:
        value = ctypes.c_float()
        self._check_ok(self.MV_CC_GetFloatValue(key, value), f"MV_CC_GetFloatValue({key!r})")
        return float(value.value)

    def getitem(self, key: str) -> Any:
        attempts = []

        def _new_int_value():
            int_value = hik.MVCC_INTVALUE()
            memset(byref(int_value), 0, sizeof(hik.MVCC_INTVALUE))
            return int_value

        def _attempt(getter_name: str, build_arg):
            getter = getattr(self, getter_name)
            arg = build_arg()
            try:
                ret = getter(key, arg)
            except Exception as exc:
                attempts.append((getter_name, exc))
                return None
            if ret != 0:
                attempts.append((getter_name, ret))
                return None
            if getter_name == "MV_CC_GetFloatValue":
                return float(arg.value)
            if getter_name == "MV_CC_GetIntValue":
                return int(arg.nCurValue)
            if getter_name == "MV_CC_GetBoolValue":
                return bool(arg.value)
            if getter_name == "MV_CC_GetEnumValue":
                return int(arg.value)
            if getter_name == "MV_CC_GetStringValue":
                value = arg.value
                if isinstance(value, bytes):
                    return value.decode(errors="ignore")
                return value
            return None

        getters = [
            ("MV_CC_GetFloatValue", ctypes.c_float),
            ("MV_CC_GetIntValue", _new_int_value),
            ("MV_CC_GetBoolValue", ctypes.c_bool),
            ("MV_CC_GetEnumValue", ctypes.c_uint32),
            ("MV_CC_GetStringValue", lambda: ctypes.create_string_buffer(256)),
        ]

        with self._lock:
            for getter_name, build_arg in getters:
                value = _attempt(getter_name, build_arg)
                if value is not None:
                    return value

        error_parts = []
        for name, result in attempts:
            if isinstance(result, Exception):
                error_parts.append(f"{name}: {type(result).__name__}: {result}")
            else:
                error_parts.append(f"{name}: 0x{result:08x}")
        raise RuntimeError(f"Cannot read camera parameter {key!r}. Attempts: {'; '.join(error_parts)}")

    def setitem(self, key: str, value: Any) -> None:
        attempts = []

        if isinstance(value, bool):
            setters = [
                ("MV_CC_SetBoolValue", bool(value)),
                ("MV_CC_SetEnumValue", int(value)),
                ("MV_CC_SetIntValue", int(value)),
            ]
        elif isinstance(value, str):
            setters = [
                ("MV_CC_SetEnumValueByString", value),
                ("MV_CC_SetStringValue", value),
            ]
        elif isinstance(value, int):
            if key in self._FLOAT_PARAM_KEYS:
                setters = [
                    ("MV_CC_SetFloatValue", float(value)),
                    ("MV_CC_SetIntValue", int(value)),
                    ("MV_CC_SetEnumValue", int(value)),
                ]
            else:
                setters = [
                    ("MV_CC_SetIntValue", int(value)),
                    ("MV_CC_SetFloatValue", float(value)),
                    ("MV_CC_SetEnumValue", int(value)),
                ]
        elif isinstance(value, float):
            setters = [
                ("MV_CC_SetFloatValue", float(value)),
                ("MV_CC_SetIntValue", int(value)),
            ]
        else:
            raise TypeError(f"Unsupported parameter type for {key!r}: {type(value).__name__}")

        with self._lock:
            for setter_name, candidate_value in setters:
                setter = getattr(self, setter_name)
                try:
                    ret = setter(key, candidate_value)
                except Exception as exc:
                    attempts.append((setter_name, exc))
                    continue
                if ret == 0:
                    return
                attempts.append((setter_name, ret))

        error_parts = []
        for name, result in attempts:
            if isinstance(result, Exception):
                error_parts.append(f"{name}: {type(result).__name__}: {result}")
            else:
                error_parts.append(f"{name}: 0x{result:08x}")
        raise RuntimeError(
            f"Cannot set camera parameter {key!r} to {value!r}. "
            f"Attempts: {'; '.join(error_parts)}"
        )

    __getitem__ = getitem
    __setitem__ = setitem

    def set_exposure(self, exposure_us: float) -> None:
        self._set_enum_by_string("ExposureAuto", "Off")
        self._set_float("ExposureTime", float(exposure_us))

    def get_exposure(self) -> float:
        return self._get_float("ExposureTime")

    def set_gain(self, gain: float) -> None:
        self._set_enum_by_string("GainAuto", "Off")
        self._set_float("Gain", float(gain))

    def get_gain(self) -> float:
        return self._get_float("Gain")

    def _configure_camera(self) -> None:
        self._set_enum("TriggerMode", hik.MV_TRIGGER_MODE_ON)
        self._set_enum("TriggerSource", hik.MV_TRIGGER_SOURCE_SOFTWARE)
        self._set_bool("AcquisitionFrameRateEnable", False)
        self._set_enum_by_string("PixelFormat", "RGB8Packed")

    def _apply_setting_items(self) -> None:
        for key, value in self._setting_items:
            self.setitem(key, value)

    def _allocate_buffers(self) -> None:
        st_param = hik.MVCC_INTVALUE()
        memset(byref(st_param), 0, sizeof(hik.MVCC_INTVALUE))
        self._check_ok(self.MV_CC_GetIntValue("PayloadSize", st_param), "MV_CC_GetIntValue('PayloadSize')")

        self._payload_size = int(st_param.nCurValue)
        self._data_buf = (ctypes.c_ubyte * self._payload_size)()
        self._frame_info = hik.MV_FRAME_OUT_INFO_EX()
        memset(byref(self._frame_info), 0, sizeof(self._frame_info))

    def _validate_rgb8_output(self, frame: np.ndarray) -> None:
        if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
            raise RuntimeError(
                "Camera must output RGB8Packed. Captured frame is not uint8 HxWx3. "
                "Raw/Bayer formats are intentionally unsupported in this version."
            )

    def __enter__(self) -> "HikCamera":
        self._check_ok(
            self.MV_CC_OpenDevice(hik.MV_ACCESS_Exclusive, 0),
            "MV_CC_OpenDevice",
        )

        self._configure_camera()
        self._apply_setting_items()
        self._allocate_buffers()
        self._check_ok(self.MV_CC_StartGrabbing(), "MV_CC_StartGrabbing")
        self._is_open = True

        frame = self.get_frame()
        self._validate_rgb8_output(frame)
        return self

    def get_frame(self) -> np.ndarray:
        if not self._is_open:
            raise RuntimeError("Camera is not open. Use `with HikCamera(...) as cam:` first.")

        with self._lock:
            self._check_ok(
                self.MV_CC_SetCommandValue("TriggerSoftware"),
                "MV_CC_SetCommandValue('TriggerSoftware')",
            )
            self._check_ok(
                self.MV_CC_GetOneFrameTimeout(
                    byref(self._data_buf),
                    self._payload_size,
                    self._frame_info,
                    self.timeout_ms,
                ),
                "MV_CC_GetOneFrameTimeout",
            )

        height = int(self._frame_info.nHeight)
        width = int(self._frame_info.nWidth)
        frame_len = int(self._frame_info.nFrameLen)
        expected_frame_len = height * width * 3

        if frame_len != expected_frame_len:
            raise RuntimeError(
                "Camera frame is not RGB8Packed. "
                f"Expected {expected_frame_len} bytes, got {frame_len}."
            )

        frame = np.ctypeslib.as_array(self._data_buf, shape=(self._payload_size,))
        return frame[:frame_len].copy().reshape(height, width, 3)

    def robust_get_frame(self) -> np.ndarray:
        return self.get_frame()

    def close(self) -> None:
        if not self._is_open:
            return

        try:
            self.MV_CC_StopGrabbing()
        finally:
            self.MV_CC_CloseDevice()
            self._is_open = False

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
        try:
            self.MV_CC_DestroyHandle()
        except Exception:
            pass


if __name__ == "__main__":
    ip = os.environ.get("HIK_CAMERA_IP")
    if not ip:
        raise SystemExit("Set HIK_CAMERA_IP to run the module demo.")

    with HikCamera(ip=ip) as cam:
        frame = cam.get_frame()
        print(f"Captured frame: shape={frame.shape}, dtype={frame.dtype}")
