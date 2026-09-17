"""Trusted local Survey checkpoints; model weights remain owned by MemNav.

Only the goal-free Survey boundary is cacheable. Each restore installs its
RGB evidence into a fresh episode directory, so subsequent query frames cannot
overwrite either the checkpoint or another run's evidence.
"""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import random
import shutil
import time
import uuid

import numpy as np

from deployment.gpu.episodic_dataset import EpisodicDatasetStore
from deployment.gpu.resident_policy import signature


SCHEMA = "memnav_survey_state_v1"
# These are resident objects or process-local paths, not episode state. All
# other agent attributes are serialized, including CPU/GPU tensors and caches.
RESIDENT = {"policy", "core", "lb", "device", "phase_b_ranker",
            "certified_relocalization_matcher", "cdec_pairwise_ranker",
            "pi3x_online_relocalizer", "buffer_root", "rgb_dir",
            "_episode_counter", "_dino_out"}


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


class SurveyStateCache:
    def __init__(self, server, config_path):
        self.server = server
        config = json.loads(Path(config_path).read_text())
        self.model_signature = signature(config)
        self.runtime = Path(config["sites"]["gpu"]["runtime_root"])
        self.root = self.runtime / "survey_state_cache"
        self.root.mkdir(parents=True, exist_ok=True)

    def context(self, payload):
        agent = self.server.agent
        dataset = EpisodicDatasetStore(self.runtime / "episodic_datasets").load(
            str(payload.get("dataset_id", "")))
        manifest_sha = digest(dataset.root / "manifest.json")
        if manifest_sha != payload.get("manifest_sha256"):
            raise ValueError("Survey manifest identity mismatch")
        if dataset.manifest.get("metadata", {}).get("collection_mode") != "local_raw_survey_v1":
            raise ValueError("State reuse is restricted to the local Survey engineering protocol")
        context = {
            "schema": SCHEMA, "dataset_id": dataset.manifest["dataset_id"],
            "manifest_sha256": manifest_sha, "model_signature": self.model_signature,
            "torch_version": str(self.server.torch.__version__),
            "camera_height": agent.camera_height,
            "camera_intrinsic": (None if agent.camera_intrinsic is None
                                 else agent.camera_intrinsic.tolist()),
            "seed": agent._last_seed, "episode_len": agent._last_episode_len,
            "frames": len(dataset.manifest["memory_frames"]),
        }
        key = hashlib.sha256(json.dumps(context, sort_keys=True).encode()).hexdigest()
        return dataset, context, self.root / key

    def check_boundary(self, frames):
        a = self.server.agent
        if a.n != frames or a._active_goal_key is not None or a._goal_start_frame:
            raise ValueError("Checkpoint requires an exact goal-free Survey boundary")
        if len(a.dino_cls) != frames or len(a.cam_pose) != frames:
            raise ValueError("Survey feature/pose frame count mismatch")

    def save(self, payload):
        started = time.monotonic()
        dataset, context, dest = self.context(payload)
        self.check_boundary(context["frames"])
        if dest.exists():
            raise ValueError("Checkpoint already exists; refusing to overwrite it")
        s = self.server; a = s.agent
        stage = self.root / (".incoming_" + uuid.uuid4().hex)
        stage.mkdir()
        # Validate the stream actually consumed this exact ordered RGB dataset.
        images = []
        (stage / "rgb").mkdir()
        for row, image in dataset.memory_frames():
            name = str(row["frame_index"]) + ".jpg"
            source = Path(a.rgb_dir) / name
            if digest(source) != row["sha256"]:
                raise ValueError("Live reconstruction RGB identity mismatch")
            shutil.copy2(source, stage / "rgb" / name)
            images.append({"path": "rgb/" + name, "sha256": row["sha256"]})
        s.torch.cuda.synchronize()
        episode_state = (
            a.export_episode_state(resident_fields=RESIDENT)
            if hasattr(a, "export_episode_state") else
            {k: v for k, v in vars(a).items() if k not in RESIDENT})
        state = {"agent": episode_state,
                 "stream": a._snapshot(), "numpy_rng": np.random.get_state(),
                 "python_rng": random.getstate(), "torch_rng": s.torch.get_rng_state(),
                 "cuda_rng": s.torch.cuda.get_rng_state_all()}
        with (stage / "state.pt").open("wb") as out:
            s.torch.save(state, out)
            out.flush(); os.fsync(out.fileno())
        receipt = {**context, "checkpoint_key": dest.name, "checkpoint_path": str(dest),
                   "state_sha256": digest(stage / "state.pt"),
                   "state_bytes": (stage / "state.pt").stat().st_size,
                   "rgb_files": images,
                   "created_utc": datetime.now(timezone.utc).isoformat(),
                   "historical_depth_cached_frames": len(a._certified_route_reference_depth_cache),
                   "goal_free": True}
        (stage / "manifest.json").write_text(json.dumps(receipt, indent=2) + "\n")
        (stage / "SEALED.sha256").write_text(digest(stage / "manifest.json") + "\n")
        stage.rename(dest)
        for p in dest.rglob("*"):
            p.chmod(p.stat().st_mode & ~0o222)
        dest.chmod(dest.stat().st_mode & ~0o222)
        return {k: v for k, v in receipt.items() if k != "rgb_files"} | {
            "cache_hit": False, "saved": True, "elapsed_s": time.monotonic() - started}

    def restore(self, payload):
        started = time.monotonic()
        dataset, context, dest = self.context(payload)
        s = self.server; a = s.agent
        if a.n != 0 or a._active_goal_key is not None:
            raise ValueError("Restore requires a freshly reset empty MemNav stream")
        if not dest.exists():
            return {"cache_hit": False, "checkpoint_key": dest.name}
        if digest(dest / "manifest.json") != (dest / "SEALED.sha256").read_text().strip():
            raise ValueError("Checkpoint manifest hash mismatch")
        receipt = json.loads((dest / "manifest.json").read_text())
        if any(receipt.get(k) != v for k, v in context.items()):
            raise ValueError("Checkpoint runtime contract mismatch")
        if digest(dest / "state.pt") != receipt["state_sha256"]:
            raise ValueError("Checkpoint state hash mismatch")
        expected = [{"path": "rgb/" + str(r["frame_index"]) + ".jpg", "sha256": r["sha256"]}
                    for r in dataset.manifest["memory_frames"]]
        if receipt["rgb_files"] != expected:
            raise ValueError("Checkpoint RGB inventory mismatch")
        for row in expected:
            if digest(dest / row["path"]) != row["sha256"]:
                raise ValueError("Checkpoint RGB hash mismatch")
        # This is our own hash-verified, local artifact, never an uploaded pickle.
        # Default device restoration preserves the original CPU/GPU placement.
        state = s.torch.load(dest / "state.pt", weights_only=False)
        if set(state["agent"]) & RESIDENT:
            raise ValueError("Checkpoint attempts to replace resident resources")
        if hasattr(a, "restore_episode_state"):
            a.restore_episode_state(state["agent"], resident_fields=RESIDENT)
        else:
            for key in set(vars(a)) - RESIDENT - set(state["agent"]):
                delattr(a, key)
            vars(a).update(state["agent"])
        a._restore(state["stream"])
        a._dino_out[0] = None
        for row in expected:
            target = Path(a.rgb_dir) / Path(row["path"]).name
            if target.exists():
                raise ValueError("Restore would overwrite existing episode RGB")
            shutil.copyfile(dest / row["path"], target)
        s.monocular_depth_transactions = {}
        np.random.set_state(state["numpy_rng"])
        random.setstate(state["python_rng"])
        s.torch.set_rng_state(state["torch_rng"])
        s.torch.cuda.set_rng_state_all(state["cuda_rng"])
        self.check_boundary(context["frames"])
        s.torch.cuda.synchronize()
        return {k: v for k, v in receipt.items() if k != "rgb_files"} | {
            "cache_hit": True, "frames_restored": a.n,
            "elapsed_s": time.monotonic() - started, "episode_buffer": a.rgb_dir}


def install(server, config_path):
    cache = SurveyStateCache(server, config_path)

    @server.app.route("/resident/survey/<action>", methods=["POST"])
    def survey_state(action):
        try:
            payload = server.request.get_json(silent=True) or {}
            if action == "save":
                return server.jsonify(cache.save(payload))
            if action == "restore":
                return server.jsonify(cache.restore(payload))
            raise ValueError("Unknown Survey state action")
        except Exception as error:
            # Hub latches uncertain state on any error; no automatic fallback
            # is permitted after a possibly partial restore.
            server.app.logger.exception("Survey checkpoint operation failed")
            return server.jsonify({"error": str(error), "reset_required": True}), 503
