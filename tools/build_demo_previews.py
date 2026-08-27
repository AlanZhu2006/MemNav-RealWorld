#!/usr/bin/env python3
"""Build a README poster and full-run time-lapse GIF from one video."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import cv2
from PIL import Image


def read_frame(capture: cv2.VideoCapture, index: int):
    capture.set(cv2.CAP_PROP_POS_FRAMES, index)
    ok, frame = capture.read()
    if not ok:
        raise RuntimeError(f"cannot decode video frame {index}")
    return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


def resize(image, width: int) -> Image.Image:
    source = Image.fromarray(image)
    height = max(2, round(source.height * width / source.width))
    if height % 2:
        height += 1
    return source.resize((width, height), Image.Resampling.LANCZOS)


def build(video: Path, poster: Path, gif: Path, width: int, frames: int, duration: float):
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open video: {video}")
    count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    if count < 2:
        raise RuntimeError("video has too few frames")
    poster_image = resize(read_frame(capture, count // 2), width)
    poster.parent.mkdir(parents=True, exist_ok=True)
    poster_image.save(poster, format="JPEG", quality=90, optimize=True)

    indices = [round(index * (count - 1) / (frames - 1)) for index in range(frames)]
    preview = [
        resize(read_frame(capture, index), width).quantize(
            colors=128, method=Image.Quantize.MEDIANCUT
        )
        for index in indices
    ]
    capture.release()
    gif.parent.mkdir(parents=True, exist_ok=True)
    frame_duration_ms = max(20, round(duration * 1000.0 / frames))
    preview[0].save(
        gif,
        save_all=True,
        append_images=preview[1:],
        duration=frame_duration_ms,
        loop=0,
        optimize=True,
        disposal=2,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--poster", type=Path, required=True)
    parser.add_argument("--gif", type=Path, required=True)
    parser.add_argument("--width", type=int, default=480)
    parser.add_argument("--frames", type=int, default=48)
    parser.add_argument("--duration-s", type=float, default=8.0)
    args = parser.parse_args()
    if args.width < 64 or args.frames < 2 or args.duration_s <= 0.0:
        parser.error("width, frames and duration must be positive")
    for output in (args.poster, args.gif):
        if output.exists():
            parser.error(f"refusing to overwrite: {output}")
    try:
        build(
            args.video,
            args.poster,
            args.gif,
            args.width,
            args.frames,
            args.duration_s,
        )
    except (OSError, RuntimeError) as error:
        parser.error(str(error))
    print(args.poster)
    print(args.gif)
    return 0


if __name__ == "__main__":
    sys.exit(main())
