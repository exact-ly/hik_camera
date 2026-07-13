#!/usr/bin/env python3

"""Minimal RGB wrapper for Hikrobot MVS camera SDK."""

from __future__ import annotations

import ctypes
from ctypes import byref, memset, sizeof
import os
import re
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
    _BAYER_FORMAT_RE = re.compile(r"^Bayer(?P<pattern>GB|GR|RG|BG)(?P<bit>8|10|12|16)(?P<packed>Packed)?$")
    _PIXEL_TYPE_FORMATS = {
        0x01080001: "Mono8",
        0x01100007: "Mono16",
        0x01080008: "BayerGR8",
        0x01080009: "BayerRG8",
        0x0108000A: "BayerGB8",
        0x0108000B: "BayerBG8",
        0x0110000C: "BayerGR10",
        0x0110000D: "BayerRG10",
        0x0110000E: "BayerGB10",
        0x0110000F: "BayerBG10",
        0x01100010: "BayerGR12",
        0x01100011: "BayerRG12",
        0x01100012: "BayerGB12",
        0x01100013: "BayerBG12",
        0x02180014: "RGB8Packed",
        0x02180015: "BGR8Packed",
        0x010C002A: "BayerGR12Packed",
        0x010C002B: "BayerRG12Packed",
        0x010C002C: "BayerGB12Packed",
        0x010C002D: "BayerBG12Packed",
    }
    _BAYER_PATTERNS = {
        "GB": "GBRG",
        "GR": "GRBG",
        "RG": "RGGB",
        "BG": "BGGR",
    }

    def __init__(
        self,
        ip: str,
        host_ip: str | None = None,
        timeout_ms: int = 40000,
        capture_format: str | None = None,
        binning: int | tuple[int, int] | None = None,
        binning_selector: str | int | None = None,
        binning_mode: str | int | None = "Average",
        setting_items: Iterable[tuple[str, Any]] | Mapping[str, Any] | None = None,
    ) -> None:
        if not ip:
            raise ValueError("`ip` is required")

        super().__init__()
        self._ip = ip
        self.host_ip = host_ip or get_host_ip_by_target_ip(ip)
        self.timeout_ms = int(timeout_ms)
        self.capture_format = capture_format
        self.binning = self._normalize_binning(binning)
        self.binning_selector = binning_selector
        self.binning_mode = binning_mode
        self.last_capture_format = None
        self.last_pixel_type = None
        self.bit = None
        self.shape = None

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
    def _normalize_binning(binning: int | tuple[int, int] | None) -> tuple[int, int] | None:
        if binning is None:
            return None
        if isinstance(binning, int):
            horizontal = vertical = binning
        else:
            try:
                horizontal, vertical = binning
            except (TypeError, ValueError) as exc:
                raise ValueError("`binning` must be an int or a two-item (horizontal, vertical) tuple") from exc

        horizontal = int(horizontal)
        vertical = int(vertical)
        if horizontal < 1 or vertical < 1:
            raise ValueError("Binning values must be positive integers")
        return horizontal, vertical

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

    def set_capture_format(self, capture_format: str) -> None:
        if self._is_open:
            raise RuntimeError("Capture format must be set before opening the camera.")
        self.capture_format = str(capture_format)

    def set_rgb(self) -> None:
        self.set_capture_format("RGB8Packed")

    def set_bayer(self, pattern: str = "GB", bit: int = 12, packed: bool = True) -> None:
        pattern = pattern.upper()
        if pattern not in {"GB", "GR", "RG", "BG"}:
            raise ValueError("Bayer pattern must be one of: GB, GR, RG, BG")
        packed_suffix = "Packed" if packed and bit % 8 else ""
        self.set_capture_format(f"Bayer{pattern}{int(bit)}{packed_suffix}")

    def set_binning(
        self,
        horizontal: int,
        vertical: int | None = None,
        selector: str | int | None = None,
        mode: str | int | None = "Average",
    ) -> None:
        if self._is_open:
            raise RuntimeError("Binning must be set before opening the camera.")
        self.binning = self._normalize_binning((horizontal, horizontal if vertical is None else vertical))
        self.binning_selector = selector
        self.binning_mode = mode

    def _apply_binning(self) -> None:
        if self.binning is None:
            return

        horizontal, vertical = self.binning
        if self.binning_selector is not None:
            self.setitem("BinningSelector", self.binning_selector)
        if self.binning_mode is not None:
            self.setitem("BinningMode", self.binning_mode)
        self.setitem("BinningHorizontal", horizontal)
        self.setitem("BinningVertical", vertical)

    def _configure_camera(self) -> None:
        self._set_enum("TriggerMode", hik.MV_TRIGGER_MODE_ON)
        self._set_enum("TriggerSource", hik.MV_TRIGGER_SOURCE_SOFTWARE)
        self._set_bool("AcquisitionFrameRateEnable", False)
        if self.capture_format is not None:
            self._set_enum_by_string("PixelFormat", self.capture_format)
        self._apply_binning()

    def _apply_setting_items(self) -> None:
        for key, value in self._setting_items:
            self.setitem(key, value)
            if key == "PixelFormat" and isinstance(value, str):
                self.capture_format = value
            elif key == "BinningSelector":
                self.binning_selector = value
            elif key == "BinningMode":
                self.binning_mode = value
            elif key == "BinningHorizontal" and self.binning is not None:
                self.binning = (int(value), self.binning[1])
            elif key == "BinningVertical" and self.binning is not None:
                self.binning = (self.binning[0], int(value))

    def _allocate_buffers(self) -> None:
        st_param = hik.MVCC_INTVALUE()
        memset(byref(st_param), 0, sizeof(hik.MVCC_INTVALUE))
        self._check_ok(self.MV_CC_GetIntValue("PayloadSize", st_param), "MV_CC_GetIntValue('PayloadSize')")

        self._payload_size = int(st_param.nCurValue)
        self._data_buf = (ctypes.c_ubyte * self._payload_size)()
        self._frame_info = hik.MV_FRAME_OUT_INFO_EX()
        memset(byref(self._frame_info), 0, sizeof(self._frame_info))

    @classmethod
    def _format_from_pixel_type(cls, pixel_type: int | None) -> str | None:
        if pixel_type is None:
            return None
        return cls._PIXEL_TYPE_FORMATS.get(int(pixel_type))

    @classmethod
    def _parse_bayer_format(cls, capture_format: str | None) -> tuple[str, int, bool] | None:
        if not capture_format:
            return None
        match = cls._BAYER_FORMAT_RE.match(capture_format)
        if not match:
            return None
        return match.group("pattern"), int(match.group("bit")), bool(match.group("packed"))

    @staticmethod
    def _bytes_to_uint16(raw: np.ndarray, height: int, width: int) -> np.ndarray:
        return raw.view("<u2").reshape(height, width).astype(np.uint16, copy=False)

    @staticmethod
    def _unpack_bayer12_packed(raw: np.ndarray, height: int, width: int) -> np.ndarray:
        raw16 = raw.astype(np.uint16)
        middle = raw16[1::3]
        left = (raw16[0::3] << 4) | (middle >> 4)
        right = (raw16[2::3] << 4) | (middle & np.uint16(0x0F))

        unpacked = np.empty(height * width, dtype=np.uint16)
        unpacked[0::2] = left
        unpacked[1::2] = right
        return unpacked.reshape(height, width)

    @staticmethod
    def _scale_to_uint8(frame: np.ndarray, bit: int) -> np.ndarray:
        if frame.dtype == np.uint8:
            return frame
        max_value = float((1 << bit) - 1)
        return np.clip(frame.astype(np.float32) * (255.0 / max_value), 0, 255).astype(np.uint8)

    @staticmethod
    def _mono_to_rgb(frame: np.ndarray, bit: int) -> np.ndarray:
        gray = HikCamera._scale_to_uint8(frame, bit)
        return np.repeat(gray[:, :, None], 3, axis=2)

    @classmethod
    def _bayer_to_rgb(cls, raw: np.ndarray, pattern: str, bit: int) -> np.ndarray:
        try:
            import cv2
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "Bayer capture requires OpenCV. Install dependency `opencv-python-headless`."
            ) from exc

        code_name = f"COLOR_Bayer{pattern}2RGB"
        code = getattr(cv2, code_name, None)
        if code is None:
            raise RuntimeError(f"OpenCV does not provide Bayer conversion {code_name}.")

        rgb = cv2.cvtColor(raw, code)
        return cls._scale_to_uint8(rgb, bit)

    def _decode_frame_to_rgb(
        self,
        raw: np.ndarray,
        height: int,
        width: int,
        frame_len: int,
        pixel_type: int | None,
    ) -> np.ndarray:
        actual_format = self._format_from_pixel_type(pixel_type) or self.capture_format
        self.last_capture_format = actual_format
        self.last_pixel_type = pixel_type

        bayer = self._parse_bayer_format(actual_format)
        if bayer is not None:
            pattern, bit, packed = bayer
            if bit == 8 and not packed:
                expected_frame_len = height * width
                if frame_len != expected_frame_len:
                    raise RuntimeError(
                        f"Invalid {actual_format} frame length. Expected {expected_frame_len} bytes, "
                        f"got {frame_len}."
                    )
                self.bit = bit
                self.shape = (height, width)
                return self._bayer_to_rgb(raw.reshape(height, width), pattern, bit)

            if bit == 12 and packed:
                expected_frame_len = height * width * 12 // 8
                if frame_len != expected_frame_len:
                    raise RuntimeError(
                        f"Invalid {actual_format} frame length. Expected {expected_frame_len} bytes, "
                        f"got {frame_len}."
                    )
                self.bit = bit
                self.shape = (height, width)
                return self._bayer_to_rgb(
                    self._unpack_bayer12_packed(raw, height, width), pattern, bit
                )

            if bit in {10, 12, 16} and not packed:
                expected_frame_len = height * width * 2
                if frame_len != expected_frame_len:
                    raise RuntimeError(
                        f"Invalid {actual_format} frame length. Expected {expected_frame_len} bytes, "
                        f"got {frame_len}."
                    )
                self.bit = bit
                self.shape = (height, width)
                return self._bayer_to_rgb(self._bytes_to_uint16(raw, height, width), pattern, bit)

            raise RuntimeError(f"Unsupported Bayer capture format: {actual_format}")

        if actual_format == "BGR8Packed":
            expected_frame_len = height * width * 3
            if frame_len != expected_frame_len:
                raise RuntimeError(
                    f"Invalid BGR8Packed frame length. Expected {expected_frame_len} bytes, got {frame_len}."
                )
            self.bit = 24
            self.shape = (height, width, 3)
            return raw.reshape(height, width, 3)[:, :, ::-1].copy()

        if actual_format == "Mono8":
            expected_frame_len = height * width
            if frame_len != expected_frame_len:
                raise RuntimeError(
                    f"Invalid Mono8 frame length. Expected {expected_frame_len} bytes, got {frame_len}."
                )
            self.bit = 8
            self.shape = (height, width)
            return self._mono_to_rgb(raw.reshape(height, width), 8)

        if actual_format == "Mono16":
            expected_frame_len = height * width * 2
            if frame_len != expected_frame_len:
                raise RuntimeError(
                    f"Invalid Mono16 frame length. Expected {expected_frame_len} bytes, got {frame_len}."
                )
            self.bit = 16
            self.shape = (height, width)
            return self._mono_to_rgb(self._bytes_to_uint16(raw, height, width), 16)

        expected_rgb_len = height * width * 3
        if frame_len == expected_rgb_len:
            self.bit = 24
            self.shape = (height, width, 3)
            self.last_capture_format = actual_format or "RGB8Packed"
            return raw.reshape(height, width, 3)

        if actual_format is None and frame_len == height * width:
            self.bit = 8
            self.shape = (height, width)
            self.last_capture_format = "Mono8"
            return self._mono_to_rgb(raw.reshape(height, width), 8)

        raise RuntimeError(
            "Unsupported camera frame format. "
            f"format={actual_format!r}, pixel_type={pixel_type!r}, "
            f"width={width}, height={height}, frame_len={frame_len}."
        )

    def get_bayer_pattern(self) -> str:
        bayer = self._parse_bayer_format(self.last_capture_format or self.capture_format)
        if bayer is None:
            raise RuntimeError("Current capture format is not a Bayer format.")
        pattern, _, _ = bayer
        return self._BAYER_PATTERNS[pattern]

    @property
    def is_raw(self) -> bool:
        return self._parse_bayer_format(self.last_capture_format or self.capture_format) is not None

    @property
    def is_bayer(self) -> bool:
        return self.is_raw

    def _validate_rgb8_output(self, frame: np.ndarray) -> None:
        if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
            raise RuntimeError(
                "get_frame() must return RGB uint8 HxWx3. "
                f"Captured frame has shape={frame.shape}, dtype={frame.dtype}."
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
        pixel_type = getattr(self._frame_info, "enPixelType", None)
        if pixel_type is not None:
            pixel_type = int(pixel_type)

        raw = np.ctypeslib.as_array(self._data_buf, shape=(self._payload_size,))
        return self._decode_frame_to_rgb(raw[:frame_len].copy(), height, width, frame_len, pixel_type)

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
