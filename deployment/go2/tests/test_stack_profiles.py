import pathlib
import sys

import pytest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from stack_profiles import (  # noqa: E402
    ARRIVAL_MODULES,
    PROFILES,
    resolve_arrival,
    resolve_profile,
    validate_combination,
)


def test_native_profile_is_an_unmodified_rgbd_baseline():
    profile = resolve_profile("native-navdp-rgbd")
    assert profile.name == "native-navdp-rgbd"
    assert profile.navigation == "original-navdp-imagegoal"
    assert profile.policy_depth == "d435-aligned-metric-depth"
    assert profile.memory == "none"
    assert profile.cec_enabled is False
    assert profile.lingbot_enabled is False
    assert profile.two_phase is False


def test_fullmono_profile_keeps_lingbot_and_cec_explicit():
    profile = resolve_profile("fullmono-lingbot-cec")
    assert profile.name == "fullmono-lingbot-cec"
    assert profile.policy_depth == "lingbot-causal-monocular-depth"
    assert profile.memory == "cec-episodic-memory"
    assert profile.cec_enabled is True
    assert profile.lingbot_enabled is True
    assert profile.two_phase is True


def test_arrival_is_independent_from_navigation_profile():
    for profile_name in PROFILES:
        profile, arrival = validate_combination(profile_name, "rgb-homography")
        assert arrival.name == "rgb-homography"
        assert arrival.name in profile.supported_arrivals


def test_only_canonical_profile_and_arrival_names_are_accepted():
    assert set(ARRIVAL_MODULES) == {
        "operator",
        "external-topic",
        "rgb-homography",
    }
    for alias in ("native", "baseline", "fullmono", "cec"):
        with pytest.raises(ValueError):
            resolve_profile(alias)
    for alias in ("none", "manual", "rgb", "external"):
        with pytest.raises(ValueError):
            resolve_arrival(alias)
