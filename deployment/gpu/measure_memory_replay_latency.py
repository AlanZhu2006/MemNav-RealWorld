#!/usr/bin/env python3
"""Replay frozen RGB HTTP requests from Jetson; no ROS or motor interface.

Measures request-to-response latency, not live camera age or navigation SR.
The GPU stack and dataset must already be prepared in an isolated namespace.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import requests


def quant(values):
    return None if not values else dict(n=len(values),
        median=float(np.median(values)), p95=float(np.percentile(values, 95)),
        maximum=float(np.max(values)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--port", type=int, default=28890)
    parser.add_argument("--limit", type=int, default=0, help="bounded integration smoke before the full replay")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((args.input / "manifest.json").read_text())
    goal = (args.input / "goal.jpg").read_bytes()
    if hashlib.sha256(goal).hexdigest() != manifest["goal_sha256"]:
        raise ValueError("frozen goal changed")
    frames = []
    for row in manifest["requests"]:
        image = (args.input / row["image"]).read_bytes()
        if hashlib.sha256(image).hexdigest() != row["sha256"]:
            raise ValueError("frozen RGB changed")
        frames.append((row, image))
    if args.limit:
        frames = frames[:args.limit]
    session = requests.Session()
    session.trust_env = False
    url = f"http://127.0.0.1:{args.port}"
    trace = (args.output / "trace.jsonl").open("x")

    def post(route, **kwargs):
        tick = time.perf_counter()
        response = session.post(url + route, timeout=(5, 600), **kwargs)
        raw = response.content
        elapsed = time.perf_counter() - tick
        if response.status_code != 200:
            raise RuntimeError(f"{route}: HTTP {response.status_code}: {raw[:1500]!r}")
        return response.json(), elapsed, len(raw)

    reset, reset_s, _ = post("/navigator_reset", json={
        "intrinsic": manifest["intrinsic"], "stop_threshold": -2.0,
        "batch_size": 1, "sample_indices": [0], "scene_name": "go2_real",
        "seed": manifest["seed"],
    })
    loaded, load_s, _ = post("/dataset/load", json={"dataset_id": manifest["dataset_id"]})
    if (loaded["manifest_sha256"] != manifest["dataset_manifest_sha256"]
            or loaded["frames_replayed"] != manifest["survey_frames"]
            or loaded["frames_restored"] != 0):
        raise RuntimeError("benchmark requires the same complete RGB replay, without a state checkpoint")
    prepare, prepare_s, _ = post("/prepare_revisit_goal", files={
        "goal": ("goal.jpg", goal, "image/jpeg"),
        "query_start": ("query_start.jpg", frames[0][1], "image/jpeg"),
    })
    if prepare["selected_goal"]["sha256"] != manifest["goal_sha256"]:
        raise RuntimeError("installed goal differs from frozen query")
    preparation = dict(reset_s=reset_s, load_s=load_s, prepare_s=prepare_s,
        reset=reset, loaded=loaded, prepare=prepare)
    (args.output / "preparation.json").write_text(json.dumps(preparation, indent=2))
    print(json.dumps({"stage": "prepared", "load_s": load_s,
                      "frames": loaded["frames_replayed"]}), flush=True)
    rows = []
    for i, (request, image) in enumerate(frames):
        result, elapsed, size = post(request["endpoint"],
            files={"image": ("image.jpg", image, "image/jpeg")},
            data={"installed_goal_sha256": manifest["goal_sha256"]})
        frame = result.get("cec_frame_idx", result.get("frame_idx"))
        if frame != request["frame"]:
            raise RuntimeError(f"RGB order changed: {frame} != {request['frame']}")
        depth = result.get("monocular_depth_receipt", {})
        if request["endpoint"] == "/imagegoal_step" and (
                depth.get("frame_index") != frame
                or depth.get("image_sha256") != request["sha256"]
                or depth.get("depth_prediction_cache_hit") is not True
                or depth.get("metric_depth_sensor_consumed") is not False):
            raise RuntimeError("planning did not consume this RGB's reused monocular depth")
        row = dict(index=i, frame=frame, endpoint=request["endpoint"],
            original_phase=request["phase"], image_sha256=request["sha256"],
            http_s=elapsed, response_bytes=size, receipt=result)
        trace.write(json.dumps(row, allow_nan=False) + "\n")
        trace.flush()
        rows.append(row)
        if i % 20 == 0:
            print(json.dumps(dict(stage="replaying", completed=i+1, total=len(frames),
                http_s=elapsed, anchor=result.get("cec_selected_anchor"),
                takeover=result.get("cec_takeover"), depth_cache=depth.get("depth_prediction_cache_hit"))), flush=True)
    trace.close()
    summary = dict(schema="gem_realworld_memory_replay_latency_v1",
        measurement="Jetson prepared JPEG request to complete GPU response via SSH",
        excluded="live sensing, JPEG encoding, arrival matcher, ROS scheduling and motor execution",
        motion_interfaces_loaded=False, input_manifest_sha256=hashlib.sha256(
            (args.input / "manifest.json").read_bytes()).hexdigest(),
        requests=len(rows), first_query_http_s=rows[0]["http_s"], metrics={})
    for label, endpoint in (("policy", "/imagegoal_step"), ("geometry", "/query_observation_step")):
        selected = [r for r in rows if r["endpoint"] == endpoint and r["original_phase"] != "warmup"]
        summary["metrics"][label] = dict(http_s=quant([r["http_s"] for r in selected]),
            takeovers=sum(r["receipt"].get("cec_takeover") is True for r in selected),
            anchors=sorted({r["receipt"]["cec_selected_anchor"] for r in selected
                            if r["receipt"].get("cec_selected_anchor") is not None}))
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
