"""Build and verify a reusable Survey on the GPU, with no robot connection."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import time

import requests


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--reset-json", required=True)
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    gpu = config["sites"]["gpu"]
    repo = Path(gpu["repository"])
    run = Path(gpu["runtime_root"]) / "survey_reconstruction" / args.dataset_id / datetime.now(
        timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    run.mkdir(parents=True, exist_ok=False)
    (run / "config.json").write_text(Path(args.config).read_text())
    reset = json.loads(args.reset_json)
    (run / "reset.json").write_text(json.dumps(reset, indent=2) + "\n")
    # The canonical launcher refuses active or unrelated sessions. It starts
    # only GPU HTTP services, which own no actuator or ROS interfaces.
    subprocess.run(["bash", str(repo / "deployment/gpu/scripts/run_policy_stack.sh"),
                    "--config", args.config], check=True)
    hub = f"http://127.0.0.1:{gpu['ports']['hub']}"

    def post(route, payload):
        response = requests.post(hub + route, json=payload, timeout=(5, 3600))
        response.raise_for_status()
        return response.json()

    def record(name, value):
        (run / name).write_text(json.dumps(value, indent=2) + "\n")
        return value

    record("reset_receipt.json", post("/navigator_reset", reset))
    print("Reconstructing sealed Survey; no goal or robot commands...", flush=True)
    started = time.monotonic()
    first = record("build.json", post("/dataset/load", {"dataset_id": args.dataset_id}))
    first_seconds = time.monotonic() - started
    print(f"Survey ready in {first_seconds:.1f}s; verifying persisted restore...", flush=True)
    post("/navigator_reset", reset)
    restored = record("restore.json", post("/dataset/load", {"dataset_id": args.dataset_id}))
    cache = restored["survey_state_cache"]
    if not cache["cache_hit"] or restored["frames_replayed"] != 0:
        raise RuntimeError("Persisted restore unexpectedly replayed Survey")
    # Real-data continuation diagnostic: append one recorded RGB, then prove
    # restoring again discards it. No planner, goal or actuator is called.
    from deployment.gpu.episodic_dataset import EpisodicDatasetStore
    dataset = EpisodicDatasetStore(Path(gpu["runtime_root"]) / "episodic_datasets").load(args.dataset_id)
    last = list(dataset.memory_frames())[-1][1]
    response = requests.post(hub + "/memory_step", files={"image": ("last.jpg", last, "image/jpeg")}, timeout=(5, 180))
    response.raise_for_status()
    continuation = record("continuation.json", response.json())
    if continuation["frame_idx"] != cache["frames_restored"]:
        raise RuntimeError("Restored stream could not continue at the expected frame")
    post("/navigator_reset", reset)
    final = record("restore_after_continuation.json", post("/dataset/load", {"dataset_id": args.dataset_id}))
    if final["frames_replayed"] != 0 or final["frames_restored"] != cache["frames_restored"]:
        raise RuntimeError("Survey checkpoint was changed by the continuation")
    health = requests.get(hub + "/healthz", timeout=10).json()
    if health["phase"] != "memory_recording" or health["active_goal_sha256"] is not None:
        raise RuntimeError("Reconstruction unexpectedly entered a goal session")
    record("ready_health.json", health)
    # Park releases only disposable working episode state. Durable Survey state
    # remains on disk and compatible GPU weights stay resident for Revisit.
    subprocess.run(["bash", str(repo / "deployment/gpu/scripts/park_policy_stack.sh"),
                    "--config", args.config], check=True)
    summary = {"dataset_id": args.dataset_id, "motion_authorized": False,
               "reconstruction_complete": True, "restore_verified": True,
               "continuation_verified": True, "query_contamination_discarded": True,
               "build_elapsed_s": first_seconds, "initial_load": first,
               "restore": restored, "gpu_weights_parked": True, "run_root": str(run)}
    record("ready.json", summary)
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
