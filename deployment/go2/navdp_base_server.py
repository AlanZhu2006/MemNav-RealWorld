#!/usr/bin/env python3
"""Minimal real-robot server for the original NavDP policy."""

from __future__ import annotations

import argparse
from functools import wraps
import json
import os
from pathlib import Path
import sys
import threading
import time

import cv2
from flask import Flask, jsonify, request
import numpy as np
from PIL import Image


REPO_ROOT = Path(__file__).resolve().parents[2]
BASELINE_DIR = REPO_ROOT / "baselines" / "navdp"
sys.path.insert(0, str(BASELINE_DIR))

from policy_agent import NavDP_Agent  # noqa: E402


app = Flask(__name__)
state_lock = threading.RLock()
navigator = None
checkpoint_path = ""
policy_device = "cuda:0"


def synchronized(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        with state_lock:
            return func(*args, **kwargs)

    return wrapper


def configure(checkpoint: str, device: str) -> None:
    global checkpoint_path, policy_device
    path = os.path.abspath(os.path.expanduser(checkpoint))
    if not os.path.isfile(path):
        raise FileNotFoundError(f"checkpoint not found: {path}")
    checkpoint_path = path
    policy_device = device


@app.route("/healthz", methods=["GET"])
def healthz():
    return jsonify({
        "status": "ok",
        "model_loaded": navigator is not None,
        "backend": "navdp",
        "device": policy_device,
    })


@app.route("/navigator_reset", methods=["POST"])
@synchronized
def reset():
    global navigator
    payload = request.get_json(force=True)
    intrinsic = np.asarray(payload.get("intrinsic"), dtype=np.float32)
    if intrinsic.shape != (3, 3) or not np.isfinite(intrinsic).all():
        raise ValueError(f"intrinsic must be a finite 3x3 matrix, got {intrinsic.shape}")
    batch_size = int(payload.get("batch_size", 1))
    if batch_size < 1:
        raise ValueError("batch_size must be positive")
    stop_threshold = float(payload.get("stop_threshold", -2.0))

    if navigator is None:
        navigator = NavDP_Agent(
            intrinsic,
            image_size=224,
            memory_size=8,
            predict_size=24,
            temporal_depth=16,
            heads=8,
            token_dim=384,
            navi_model=checkpoint_path,
            device=policy_device,
        )
    navigator.reset(batch_size, stop_threshold)
    return jsonify({"algo": "navdp"})


def _decode_rgb_depth():
    if navigator is None:
        raise RuntimeError("call /navigator_reset before inference")
    if "image" not in request.files or "depth" not in request.files:
        raise ValueError("multipart request must contain image and depth")

    rgb = Image.open(request.files["image"].stream).convert("RGB")
    rgb = cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR)
    depth = Image.open(request.files["depth"].stream).convert("I")
    depth = np.asarray(depth, dtype=np.float32) / 10000.0
    if rgb.shape[:2] != depth.shape[:2]:
        raise ValueError(f"RGB/depth shape mismatch: {rgb.shape[:2]} vs {depth.shape[:2]}")

    batch_size = int(navigator.batch_size)
    rgb = rgb.reshape((batch_size, -1, rgb.shape[1], 3))
    depth = depth[:, :, np.newaxis].reshape((batch_size, -1, depth.shape[1], 1))
    return rgb, depth


def _response(result):
    trajectory, all_trajectory, all_values, _ = result
    return jsonify({
        "trajectory": trajectory.tolist(),
        "all_trajectory": all_trajectory.tolist(),
        "all_values": all_values.tolist(),
    })


@app.route("/pointgoal_step", methods=["POST"])
@synchronized
def pointgoal_step():
    rgb, depth = _decode_rgb_depth()
    goal_data = json.loads(request.form.get("goal_data", "{}"))
    goal_x = np.asarray(goal_data.get("goal_x"), dtype=np.float32)
    goal_y = np.asarray(goal_data.get("goal_y"), dtype=np.float32)
    if goal_x.shape != goal_y.shape or goal_x.size != int(navigator.batch_size):
        raise ValueError("goal_x and goal_y must match the policy batch size")
    goal = np.stack((goal_x, goal_y, np.zeros_like(goal_x)), axis=1)
    return _response(navigator.step_pointgoal(goal, rgb, depth))


@app.route("/nogoal_step", methods=["POST"])
@synchronized
def nogoal_step():
    rgb, depth = _decode_rgb_depth()
    return _response(navigator.step_nogoal(rgb, depth))


@app.route("/imagegoal_step", methods=["POST"])
@synchronized
def imagegoal_step():
    rgb, depth = _decode_rgb_depth()
    if "goal" not in request.files:
        raise ValueError("multipart request must contain goal")
    goal = Image.open(request.files["goal"].stream).convert("RGB")
    goal = cv2.cvtColor(np.asarray(goal), cv2.COLOR_RGB2BGR)
    batch_size = int(navigator.batch_size)
    goal = goal.reshape((batch_size, -1, goal.shape[1], 3))
    return _response(navigator.step_imagegoal(goal, rgb, depth))


@app.route("/navdp_step_ip_mixgoal", methods=["POST"])
@synchronized
def mixed_image_pointgoal_step():
    rgb, depth = _decode_rgb_depth()
    if "image_goal" not in request.files:
        raise ValueError("multipart request must contain image_goal")
    goal_data = json.loads(request.form.get("goal_data", "{}"))
    goal_x = np.asarray(goal_data.get("goal_x"), dtype=np.float32)
    goal_y = np.asarray(goal_data.get("goal_y"), dtype=np.float32)
    if goal_x.shape != goal_y.shape or goal_x.size != int(navigator.batch_size):
        raise ValueError("goal_x and goal_y must match the policy batch size")
    point_goal = np.stack((goal_x, goal_y, np.zeros_like(goal_x)), axis=1)
    image_goal = Image.open(request.files["image_goal"].stream).convert("RGB")
    image_goal = cv2.cvtColor(np.asarray(image_goal), cv2.COLOR_RGB2BGR)
    batch_size = int(navigator.batch_size)
    image_goal = image_goal.reshape((batch_size, -1, image_goal.shape[1], 3))
    return _response(navigator.step_point_image_goal(point_goal, image_goal, rgb, depth))


@app.route("/shutdown", methods=["POST"])
def shutdown():
    shutdown_function = request.environ.get("werkzeug.server.shutdown")

    def exit_after_response() -> None:
        time.sleep(0.15)
        if shutdown_function is not None:
            shutdown_function()
        else:
            os._exit(0)

    threading.Thread(target=exit_after_response, daemon=True).start()
    return jsonify({"status": "ok", "message": "server shutting down"})


def main() -> None:
    parser = argparse.ArgumentParser(description="Original NavDP real-robot policy server")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8888)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    configure(args.checkpoint, args.device)
    app.run(host=args.host, port=args.port, threaded=False)


if __name__ == "__main__":
    main()
