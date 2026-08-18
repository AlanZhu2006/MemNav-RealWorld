#!/usr/bin/env python3
"""Load and save RGB image goals without changing their channel order."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


def validate_rgb_image(image: np.ndarray) -> np.ndarray:
    rgb = np.asarray(image)
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f"RGB image must have shape (H, W, 3), got {rgb.shape}")
    if rgb.size == 0:
        raise ValueError("RGB image is empty")
    return np.asarray(rgb, dtype=np.uint8)


def validate_depth_m(depth_m: np.ndarray) -> np.ndarray:
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim == 3 and depth.shape[2] == 1:
        depth = depth[..., 0]
    if depth.ndim != 2:
        raise ValueError(f"depth image must have shape (H, W), got {depth.shape}")
    if depth.size == 0:
        raise ValueError("depth image is empty")
    return np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)


def depth_array_to_meters(
    depth: np.ndarray, integer_scale_m: float = 0.001
) -> np.ndarray:
    source = np.asarray(depth)
    if np.issubdtype(source.dtype, np.integer):
        return validate_depth_m(source.astype(np.float32) * float(integer_scale_m))
    return validate_depth_m(source)


def load_rgb_image(path: str | Path) -> np.ndarray:
    source = Path(path).expanduser().resolve()
    image = cv2.imread(str(source), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"image goal could not be read: {source}")
    return validate_rgb_image(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))


def save_rgb_image(path: str | Path, image: np.ndarray) -> Path:
    destination = Path(path).expanduser().resolve()
    if not destination.suffix:
        raise ValueError("image goal output must have an image extension")
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(
        f".{destination.stem}.tmp{destination.suffix}"
    )
    bgr = cv2.cvtColor(validate_rgb_image(image), cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(str(temporary), bgr):
        raise RuntimeError(f"failed to write image goal: {temporary}")
    temporary.replace(destination)
    return destination


def load_depth_image(path: str | Path, scale_m: float = 0.001) -> np.ndarray:
    source = Path(path).expanduser().resolve()
    encoded = cv2.imread(str(source), cv2.IMREAD_UNCHANGED)
    if encoded is None:
        raise FileNotFoundError(f"depth image goal could not be read: {source}")
    if encoded.ndim != 2:
        raise ValueError(f"encoded depth image must be single-channel, got {encoded.shape}")
    return validate_depth_m(encoded.astype(np.float32) * float(scale_m))


def save_depth_image(
    path: str | Path, depth_m: np.ndarray, scale_m: float = 0.001
) -> Path:
    if scale_m <= 0.0:
        raise ValueError("depth scale must be positive")
    destination = Path(path).expanduser().resolve()
    if destination.suffix.lower() != ".png":
        raise ValueError("depth image output must use PNG")
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(
        f".{destination.stem}.tmp{destination.suffix}"
    )
    depth = validate_depth_m(depth_m)
    encoded = np.clip(depth / float(scale_m), 0.0, 65535.0).astype(np.uint16)
    if not cv2.imwrite(str(temporary), encoded):
        raise RuntimeError(f"failed to write depth image goal: {temporary}")
    temporary.replace(destination)
    return destination
