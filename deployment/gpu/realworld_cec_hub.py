#!/usr/bin/env python3
"""Single-entry real-world bridge for CEC and frozen NavDP.

The Go2 client sees the ordinary NavDP ``/navigator_reset`` and
``/imagegoal_step`` contract.  Internally this process performs the frozen
Certified Episodic Compass decision and delegates control to either native
ImageGoal NavDP or the mixed image/PointGoal controller.

This process owns no actuator interface.  It is intended to listen on
loopback and be reached through an SSH local-forward from the robot computer.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import io
import json
import math
import threading
from typing import Any, Mapping

from flask import Flask, jsonify, request
import requests

from deployment.gpu.revisit_bearing_adapter import adapt_revisit_pointgoal


PROTOCOL_VERSION = 1
POINTGOAL_UNITS = "lingbot_raw_direction_only"
PROPOSAL_ORDER = "geometry_first"


class HybridBackendError(RuntimeError):
    """A stateful upstream failed and the session can no longer continue."""


@dataclass(frozen=True)
class UpstreamConfig:
    memnav_url: str
    navdp_url: str
    connect_timeout_s: float = 3.0
    request_timeout_s: float = 180.0

    @property
    def timeout(self) -> tuple[float, float]:
        return (self.connect_timeout_s, self.request_timeout_s)


def _json_object(response: requests.Response, label: str) -> dict[str, Any]:
    response.raise_for_status()
    payload = response.json()
    if not isinstance(payload, dict):
        raise HybridBackendError(f"{label} returned non-object JSON")
    return payload


def _file(name: str, payload: bytes, media_type: str) -> tuple[str, io.BytesIO, str]:
    return (name, io.BytesIO(payload), media_type)


def _finite_intrinsic(value: Any) -> list[list[float]]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError("intrinsic must be a finite 3x3 matrix")
    matrix: list[list[float]] = []
    for row in value:
        if not isinstance(row, list) or len(row) != 3:
            raise ValueError("intrinsic must be a finite 3x3 matrix")
        parsed = [float(item) for item in row]
        if not all(math.isfinite(item) for item in parsed):
            raise ValueError("intrinsic must be a finite 3x3 matrix")
        matrix.append(parsed)
    return matrix


class CecHybridRouter:
    """Stateful exactly-one-probe CEC router with a native safety fallback."""

    def __init__(
        self,
        config: UpstreamConfig,
        *,
        session: requests.Session | None = None,
    ) -> None:
        self.config = config
        self.session = session or requests.Session()
        self.initialized = False
        self.memory_degraded = False
        self.native_state_uncertain = False
        self.step_index = 0

    def reset(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        intrinsic = _finite_intrinsic(payload.get("intrinsic"))
        navdp_payload = dict(payload)
        navdp_payload["intrinsic"] = intrinsic
        memnav_payload = {
            "camera_height": float(payload.get("camera_height", 0.5)),
            "camera_intrinsic": intrinsic,
            "seed": payload.get("seed"),
            "episode_len": payload.get("episode_len"),
        }
        self.initialized = False
        self.memory_degraded = False
        self.native_state_uncertain = False
        self.step_index = 0
        try:
            memnav = _json_object(
                self.session.post(
                    f"{self.config.memnav_url}/navigator_reset",
                    json=memnav_payload,
                    timeout=self.config.timeout,
                ),
                "MemNav reset",
            )
            navdp = _json_object(
                self.session.post(
                    f"{self.config.navdp_url}/navigator_reset",
                    json=navdp_payload,
                    timeout=self.config.timeout,
                ),
                "NavDP reset",
            )
        except Exception as error:
            raise HybridBackendError(
                f"atomic reset failed: {type(error).__name__}: {error}"
            ) from error
        self.initialized = True
        certificate_status = memnav.get("certified_relocalization")
        certificate_enabled = (
            bool(certificate_status.get("enabled"))
            if isinstance(certificate_status, dict)
            else bool(certificate_status)
        )
        return {
            "algo": "cec_hybrid_navdp",
            "protocol_version": PROTOCOL_VERSION,
            "memnav_algo": memnav.get("algo"),
            "navdp_algo": navdp.get("algo", "navdp"),
            "certificate_enabled": certificate_enabled,
        }

    def _native_plan(
        self,
        image: bytes,
        goal: bytes,
        depth: bytes,
        form: Mapping[str, str],
    ) -> dict[str, Any]:
        try:
            return _json_object(
                self.session.post(
                    f"{self.config.navdp_url}/imagegoal_step",
                    files={
                        "image": _file("image.jpg", image, "image/jpeg"),
                        "goal": _file("goal.jpg", goal, "image/jpeg"),
                        "depth": _file("depth.png", depth, "image/png"),
                    },
                    data=dict(form),
                    timeout=self.config.timeout,
                ),
                "native NavDP step",
            )
        except Exception as error:
            self.native_state_uncertain = True
            raise HybridBackendError(
                "native NavDP state is uncertain; reset is required: "
                f"{type(error).__name__}: {error}"
            ) from error

    def _mixed_plan(
        self,
        image: bytes,
        goal: bytes,
        depth: bytes,
        pointgoal: tuple[float, float],
        form: Mapping[str, str],
    ) -> dict[str, Any]:
        data = dict(form)
        data["goal_data"] = json.dumps({
            "goal_x": [float(pointgoal[0])],
            "goal_y": [float(pointgoal[1])],
        })
        try:
            return _json_object(
                self.session.post(
                    f"{self.config.navdp_url}/navdp_step_ip_mixgoal",
                    files={
                        "image": _file("image.jpg", image, "image/jpeg"),
                        "image_goal": _file("goal.jpg", goal, "image/jpeg"),
                        "depth": _file("depth.png", depth, "image/png"),
                    },
                    data=data,
                    timeout=self.config.timeout,
                ),
                "mixed NavDP step",
            )
        except Exception as error:
            self.native_state_uncertain = True
            raise HybridBackendError(
                "mixed NavDP state is uncertain; reset is required: "
                f"{type(error).__name__}: {error}"
            ) from error

    def plan_imagegoal(
        self,
        *,
        image: bytes,
        goal: bytes,
        depth: bytes,
        form: Mapping[str, str] | None = None,
    ) -> dict[str, Any]:
        if not self.initialized:
            raise HybridBackendError("router is not initialized")
        if self.native_state_uncertain:
            raise HybridBackendError("native state is uncertain; reset is required")
        if not image or not goal or not depth:
            raise ValueError("image, goal, and depth are required")
        form = dict(form or {})
        self.step_index += 1

        # A failed memory process must never prevent native navigation.  Once
        # degraded, however, CEC remains disabled until an explicit reset so
        # it cannot reason over a history with missing observations.
        if self.memory_degraded:
            result = self._native_plan(image, goal, depth, form)
            result.update({
                "cec_takeover": False,
                "cec_reason": "memory_session_degraded",
                "cec_step_index": self.step_index,
            })
            return result

        try:
            probe = _json_object(
                self.session.post(
                    f"{self.config.memnav_url}/retrieval_probe_step",
                    files={
                        "image": _file("image.jpg", image, "image/jpeg"),
                        "goal": _file("goal.jpg", goal, "image/jpeg"),
                    },
                    data=form,
                    timeout=self.config.timeout,
                ),
                "CEC retrieval probe",
            )
        except Exception as error:
            self.memory_degraded = True
            result = self._native_plan(image, goal, depth, form)
            result.update({
                "cec_takeover": False,
                "cec_reason": "retrieval_probe_failure_native_only",
                "cec_error": f"{type(error).__name__}: {error}",
                "cec_step_index": self.step_index,
            })
            return result

        candidates = probe.get("certified_visual_candidates")
        if not isinstance(candidates, list):
            candidates = []
        try:
            certificate = _json_object(
                self.session.post(
                    f"{self.config.memnav_url}/certified_relocalize",
                    files={"goal": _file("goal.jpg", goal, "image/jpeg")},
                    data={
                        "candidates": json.dumps(candidates),
                        "proposal_order": PROPOSAL_ORDER,
                        "graph_rescue": "0",
                        "learned_rescue": "0",
                    },
                    timeout=self.config.timeout,
                ),
                "CEC certificate",
            )
        except Exception as error:
            certificate = {
                "ok": False,
                "accepted": False,
                "reason": "certificate_endpoint_failure",
                "error": f"{type(error).__name__}: {error}",
            }

        active = bool(certificate.get("ok") is True and certificate.get("accepted") is True)
        units = certificate.get("pointgoal_units")
        if active and units != POINTGOAL_UNITS:
            active = False
            certificate["reason"] = "invalid_scale_free_output_contract"
        decision = adapt_revisit_pointgoal(
            mode="verified_bearing_v1",
            router_active=active,
            pointgoal=certificate.get("aux_pose"),
            source="lightglue_lingbot_pnp_v2_scale_free",
            pointgoal_units=POINTGOAL_UNITS,
        )
        if decision.takeover:
            assert decision.controller_pointgoal is not None
            result = self._mixed_plan(
                image, goal, depth, decision.controller_pointgoal, form
            )
            controller = "navdp_image_point_mix"
        else:
            result = self._native_plan(image, goal, depth, form)
            controller = "navdp_image_router"
        result.update(decision.audit_dict())
        result.update({
            "cec_takeover": decision.takeover,
            "cec_reason": certificate.get("reason", decision.reason),
            "cec_controller": controller,
            "cec_step_index": self.step_index,
            "cec_frame_idx": probe.get("frame_idx"),
            "cec_selected_anchor": certificate.get("selected_anchor"),
            "cec_certificate": certificate.get("certificate"),
            "cec_relocalization_ms": certificate.get("relocalization_ms"),
        })
        if certificate.get("error"):
            result["cec_error"] = certificate["error"]
        return result


def create_app(router: CecHybridRouter) -> Flask:
    app = Flask(__name__)
    call_lock = threading.Lock()

    @app.get("/healthz")
    def healthz():
        return jsonify({
            "ok": True,
            "algo": "cec_hybrid_navdp",
            "protocol_version": PROTOCOL_VERSION,
            "initialized": router.initialized,
            "memory_degraded": router.memory_degraded,
            "native_state_uncertain": router.native_state_uncertain,
        })

    @app.post("/navigator_reset")
    def navigator_reset():
        if not call_lock.acquire(blocking=False):
            return jsonify({"error": "hub_busy"}), 409
        try:
            try:
                return jsonify(router.reset(request.get_json(silent=True) or {}))
            except ValueError as error:
                return jsonify({"error": str(error)}), 400
            except HybridBackendError as error:
                return jsonify({"error": str(error), "reset_required": True}), 503
        finally:
            call_lock.release()

    @app.post("/imagegoal_step")
    def imagegoal_step():
        missing = [name for name in ("image", "goal", "depth") if name not in request.files]
        if missing:
            return jsonify({"error": f"missing files: {', '.join(missing)}"}), 400
        if not call_lock.acquire(blocking=False):
            return jsonify({"error": "hub_busy"}), 409
        try:
            try:
                result = router.plan_imagegoal(
                    image=request.files["image"].read(),
                    goal=request.files["goal"].read(),
                    depth=request.files["depth"].read(),
                    form=request.form.to_dict(flat=True),
                )
                app.logger.info(
                    "cec_plan step=%s takeover=%s controller=%s reason=%s "
                    "frame=%s anchor=%s relocalization_ms=%s",
                    result.get("cec_step_index"),
                    result.get("cec_takeover"),
                    result.get("cec_controller"),
                    result.get("cec_reason"),
                    result.get("cec_frame_idx"),
                    result.get("cec_selected_anchor"),
                    result.get("cec_relocalization_ms"),
                )
                return jsonify(result)
            except ValueError as error:
                return jsonify({"error": str(error)}), 400
            except HybridBackendError as error:
                return jsonify({"error": str(error), "reset_required": True}), 503
        finally:
            call_lock.release()

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18889)
    parser.add_argument("--memnav-url", default="http://127.0.0.1:18888")
    parser.add_argument("--navdp-url", default="http://127.0.0.1:8888")
    parser.add_argument("--connect-timeout-s", type=float, default=3.0)
    parser.add_argument("--request-timeout-s", type=float, default=180.0)
    args = parser.parse_args()
    if args.host not in {"127.0.0.1", "::1", "localhost"}:
        parser.error("real-world hub must bind to loopback; use an SSH tunnel")
    router = CecHybridRouter(UpstreamConfig(
        memnav_url=args.memnav_url.rstrip("/"),
        navdp_url=args.navdp_url.rstrip("/"),
        connect_timeout_s=args.connect_timeout_s,
        request_timeout_s=args.request_timeout_s,
    ))
    create_app(router).run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
