from pathlib import Path
from types import SimpleNamespace
import sys

import cv2
import numpy as np
import pytest


REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "deployment/go2"))

from foxglove_image_relay import (  # noqa: E402
    colorize_depth_preview,
    depth_message_to_u16,
    encode_jpeg,
    resize_rgb_preview,
    rgb_message_to_bgr,
)


def _message(
    array: np.ndarray,
    encoding: str,
    *,
    step: int | None = None,
    is_bigendian: bool = False,
) -> SimpleNamespace:
    height, width = array.shape[:2]
    return SimpleNamespace(
        width=width,
        height=height,
        encoding=encoding,
        step=step if step is not None else array.strides[0],
        is_bigendian=is_bigendian,
        data=array.tobytes(),
    )


def test_rgb_preview_converts_resizes_and_encodes_jpeg():
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255
    bgr = rgb_message_to_bgr(_message(rgb, "rgb8"))
    assert np.array_equal(bgr[0, 0], [0, 0, 255])

    preview = resize_rgb_preview(bgr, 640, 360)
    decoded = cv2.imdecode(
        np.frombuffer(encode_jpeg(preview, 75), dtype=np.uint8), cv2.IMREAD_COLOR
    )
    assert decoded.shape == (360, 640, 3)
    assert int(decoded[180, 320, 2]) > 240


def test_rgb_decoder_respects_padded_rows():
    rows = np.zeros((2, 8), dtype=np.uint8)
    rows[:, :6] = np.array([255, 0, 0, 0, 255, 0], dtype=np.uint8)
    message = SimpleNamespace(
        width=2,
        height=2,
        encoding="rgb8",
        step=8,
        is_bigendian=False,
        data=rows.tobytes(),
    )
    bgr = rgb_message_to_bgr(message)
    assert bgr.shape == (2, 2, 3)
    assert np.array_equal(bgr[0, 0], [0, 0, 255])
    assert np.array_equal(bgr[0, 1], [0, 255, 0])


def test_depth_preview_preserves_invalid_mask_and_colorizes_range():
    depth = np.array([[0, 200], [1000, 4000]], dtype=np.uint16)
    decoded = depth_message_to_u16(_message(depth, "16UC1"))
    assert np.array_equal(decoded, depth)

    preview = colorize_depth_preview(decoded, 2, 2, 200, 4000)
    assert preview.shape == (2, 2, 3)
    assert np.array_equal(preview[0, 0], [0, 0, 0])
    assert np.any(preview[0, 1] != 0)
    assert np.any(preview[1, 1] != preview[0, 1])


@pytest.mark.parametrize(
    ("encoding", "decoder"),
    [("mono8", rgb_message_to_bgr), ("32FC1", depth_message_to_u16)],
)
def test_unsupported_encodings_fail_closed(encoding, decoder):
    array = np.zeros((2, 2), dtype=np.uint8)
    with pytest.raises(ValueError, match="unsupported"):
        decoder(_message(array, encoding))
