#!/usr/bin/env python3
"""Declarative real-robot stack profiles and arrival-module contracts.

This module deliberately contains no ROS, model or motor code.  It gives the
launchers one vocabulary for describing which navigation, depth, memory and
termination modules are active, so a backend change does not silently change
the rest of the experiment.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from typing import Dict, Tuple


@dataclass(frozen=True)
class StackProfile:
    name: str
    launcher: str
    navigation: str
    policy_depth: str
    memory: str
    cec_enabled: bool
    lingbot_enabled: bool
    two_phase: bool
    description: str
    aliases: Tuple[str, ...] = ()
    supported_arrivals: Tuple[str, ...] = (
        "operator",
        "external-topic",
        "rgb-homography",
    )


@dataclass(frozen=True)
class ArrivalModule:
    name: str
    process: str
    authority: str
    description: str


PROFILES: Dict[str, StackProfile] = {
    "native-navdp-rgbd": StackProfile(
        name="native-navdp-rgbd",
        launcher="local",
        navigation="original-navdp-imagegoal",
        policy_depth="d435-aligned-metric-depth",
        memory="none",
        cec_enabled=False,
        lingbot_enabled=False,
        two_phase=False,
        description=(
            "Original NavDP ImageGoal baseline with live D435i RGB-D. "
            "No CEC, MemNav or LingBot policy input."
        ),
        aliases=("native", "baseline"),
    ),
    "fullmono-lingbot-cec": StackProfile(
        name="fullmono-lingbot-cec",
        launcher="offboard",
        navigation="frozen-navdp-imagegoal",
        policy_depth="lingbot-causal-monocular-depth",
        memory="cec-episodic-memory",
        cec_enabled=True,
        lingbot_enabled=True,
        two_phase=True,
        description=(
            "Two-machine Full-Mono stack: LingBot depth, CEC memory/proof "
            "and frozen NavDP trajectory generation."
        ),
        aliases=("fullmono", "cec"),
    ),
}


ARRIVAL_MODULES: Dict[str, ArrivalModule] = {
    "operator": ArrivalModule(
        name="operator",
        process="none",
        authority="operator",
        description="No autonomous arrival process; operator terminates the run.",
    ),
    "external-topic": ArrivalModule(
        name="external-topic",
        process="none",
        authority="/navdp/arrival",
        description=(
            "An independent process (tag, SLAM or evaluator) owns arrival and "
            "publishes std_msgs/Bool on /navdp/arrival."
        ),
    ),
    "rgb-homography": ArrivalModule(
        name="rgb-homography",
        process="run_rgb_goal_arrival.sh",
        authority="temporary-rgb-geometry",
        description=(
            "Experimental RGB SIFT/homography gate. It is a termination module, "
            "not part of NavDP planning."
        ),
    ),
}


def resolve_profile(name: str) -> StackProfile:
    normalized = str(name).strip().lower()
    if normalized in PROFILES:
        return PROFILES[normalized]
    for profile in PROFILES.values():
        if normalized in profile.aliases:
            return profile
    choices = ", ".join(sorted(PROFILES))
    raise ValueError(f"unknown stack profile {name!r}; choose: {choices}")


def resolve_arrival(name: str) -> ArrivalModule:
    normalized = str(name).strip().lower()
    aliases = {
        "none": "operator",
        "manual": "operator",
        "rgb": "rgb-homography",
        "external": "external-topic",
    }
    normalized = aliases.get(normalized, normalized)
    if normalized not in ARRIVAL_MODULES:
        choices = ", ".join(sorted(ARRIVAL_MODULES))
        raise ValueError(f"unknown arrival module {name!r}; choose: {choices}")
    return ARRIVAL_MODULES[normalized]


def validate_combination(profile_name: str, arrival_name: str) -> tuple:
    profile = resolve_profile(profile_name)
    arrival = resolve_arrival(arrival_name)
    if arrival.name not in profile.supported_arrivals:
        raise ValueError(
            f"arrival {arrival.name!r} is not supported by {profile.name!r}"
        )
    return profile, arrival


def profile_payload(profile: StackProfile) -> dict:
    payload = asdict(profile)
    payload["aliases"] = list(profile.aliases)
    payload["supported_arrivals"] = list(profile.supported_arrivals)
    return payload


def _print_list() -> None:
    print("PROFILE\tPOLICY DEPTH\tMEMORY\tDESCRIPTION")
    for profile in PROFILES.values():
        print(
            f"{profile.name}\t{profile.policy_depth}\t{profile.memory}\t"
            f"{profile.description}"
        )
    print("\nARRIVAL\tAUTHORITY\tDESCRIPTION")
    for module in ARRIVAL_MODULES.values():
        print(f"{module.name}\t{module.authority}\t{module.description}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect Go2 stack profiles")
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("list")
    show = subparsers.add_parser("show")
    show.add_argument("profile")
    get = subparsers.add_parser("get")
    get.add_argument("profile")
    get.add_argument("field")
    validate = subparsers.add_parser("validate")
    validate.add_argument("profile")
    validate.add_argument("arrival")
    args = parser.parse_args()

    try:
        if args.command == "list":
            _print_list()
            return 0
        if args.command == "show":
            profile = resolve_profile(args.profile)
            payload = profile_payload(profile)
            payload["arrival_modules"] = {
                name: asdict(ARRIVAL_MODULES[name])
                for name in profile.supported_arrivals
            }
            print(json.dumps(payload, ensure_ascii=False, indent=2))
            return 0
        if args.command == "get":
            payload = profile_payload(resolve_profile(args.profile))
            if args.field not in payload:
                raise ValueError(f"unknown profile field {args.field!r}")
            value = payload[args.field]
            print(json.dumps(value) if isinstance(value, (list, dict)) else value)
            return 0
        profile, arrival = validate_combination(args.profile, args.arrival)
        print(f"{profile.name} {arrival.name}")
        return 0
    except ValueError as exc:
        parser.error(str(exc))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
