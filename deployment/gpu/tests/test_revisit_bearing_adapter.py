import math
import unittest

from deployment.gpu.revisit_bearing_adapter import (
    VERIFIED_BEARING_RADIUS_M,
    adapt_revisit_pointgoal,
    validate_revisit_adapter_configuration,
)


class RevisitBearingAdapterTest(unittest.TestCase):
    def test_verified_bearing_removes_distance_and_preserves_direction(self):
        short = adapt_revisit_pointgoal(
            mode="verified_bearing_v1",
            router_active=True,
            pointgoal=[-1.0, 2.0],
        )
        long = adapt_revisit_pointgoal(
            mode="verified_bearing_v1",
            router_active=True,
            pointgoal=[-10.0, 20.0],
        )
        self.assertTrue(short.takeover)
        self.assertEqual(short.controller_pointgoal, long.controller_pointgoal)
        self.assertAlmostEqual(
            math.hypot(*short.controller_pointgoal),
            VERIFIED_BEARING_RADIUS_M,
        )
        self.assertAlmostEqual(
            short.unit_bearing[0] * short.controller_pointgoal[1]
            - short.unit_bearing[1] * short.controller_pointgoal[0],
            0.0,
        )

    def test_inactive_router_abstains_even_with_a_valid_vector(self):
        decision = adapt_revisit_pointgoal(
            mode="verified_bearing_v1",
            router_active=False,
            pointgoal=[1.0, 0.0],
        )
        self.assertFalse(decision.takeover)
        self.assertEqual(decision.reason, "router_inactive")
        self.assertEqual(decision.controller_contract, "native_imagegoal")
        self.assertIsNone(decision.controller_pointgoal)

    def test_invalid_and_zero_evidence_fail_closed(self):
        cases = (
            (None, "missing_pointgoal"),
            ([1.0], "invalid_pointgoal"),
            ({"x": 1.0, "y": 2.0}, "invalid_pointgoal"),
            ([float("nan"), 0.0], "invalid_pointgoal"),
            ([0.0, 0.0], "zero_bearing"),
        )
        for pointgoal, reason in cases:
            with self.subTest(pointgoal=pointgoal):
                decision = adapt_revisit_pointgoal(
                    mode="verified_bearing_v1",
                    router_active=True,
                    pointgoal=pointgoal,
                )
                self.assertFalse(decision.takeover)
                self.assertEqual(decision.reason, reason)

    def test_legacy_mode_is_an_exact_metric_interface(self):
        decision = adapt_revisit_pointgoal(
            mode="legacy_metric",
            router_active=True,
            pointgoal=[3.0, -4.0],
        )
        self.assertTrue(decision.takeover)
        self.assertEqual(decision.controller_pointgoal, (3.0, -4.0))
        self.assertEqual(decision.controller_distance_m, 5.0)
        self.assertIsNone(
            decision.audit_dict()["memory_pointgoal_fixed_radius_m"])

    def test_certified_raw_vector_contributes_direction_but_not_distance(self):
        decision = adapt_revisit_pointgoal(
            mode="verified_bearing_v1",
            router_active=True,
            pointgoal=[3.0, -4.0],
            source="lightglue_lingbot_pnp_v2_scale_free",
            pointgoal_units="lingbot_raw_direction_only",
        )
        self.assertTrue(decision.takeover)
        self.assertEqual(decision.reason, "verified_scale_free_bearing")
        self.assertEqual(decision.controller_pointgoal, (1.5, -2.0))
        self.assertEqual(decision.raw_pointgoal_norm, 5.0)
        self.assertIsNone(decision.raw_distance_m)
        self.assertEqual(decision.controller_distance_m, 2.5)
        self.assertEqual(
            decision.controller_contract, "mixed_imagegoal_pointgoal")

    def test_raw_fixed_bearing_is_same_controller_input_without_verification_claim(self):
        decision = adapt_revisit_pointgoal(
            mode="raw_fixed_bearing_v1",
            router_active=True,
            pointgoal=[3.0, -4.0],
            source="raw_dino_top1_metric_pose",
        )
        self.assertTrue(decision.takeover)
        self.assertEqual(decision.reason, "raw_uncertified_fixed_bearing")
        self.assertEqual(decision.controller_pointgoal, (1.5, -2.0))
        self.assertEqual(decision.raw_distance_m, 5.0)
        self.assertEqual(decision.controller_distance_m, 2.5)
        self.assertEqual(
            decision.audit_dict()["memory_pointgoal_fixed_radius_m"], 2.5
        )

    def test_front_support_preserves_forward_metric_pointgoal(self):
        decision = adapt_revisit_pointgoal(
            mode="navdp_front_support_v1",
            router_active=True,
            pointgoal=[0.0, -4.0],
        )
        self.assertTrue(decision.takeover)
        self.assertEqual(decision.reason, "pointgoal_inside_navdp_support")
        self.assertEqual(decision.controller_pointgoal, (0.0, -4.0))
        self.assertEqual(decision.controller_distance_m, 4.0)

    def test_front_support_falls_back_before_navdp_clips_behind_goal(self):
        decision = adapt_revisit_pointgoal(
            mode="navdp_front_support_v1",
            router_active=True,
            pointgoal=[-0.001, 3.0],
        )
        self.assertFalse(decision.takeover)
        self.assertEqual(decision.reason, "pointgoal_behind_navdp_support")
        self.assertEqual(decision.raw_pointgoal, (-0.001, 3.0))
        self.assertIsNone(decision.controller_pointgoal)
        self.assertEqual(decision.controller_contract, "native_imagegoal")

    def test_front_support_rejects_zero_vector(self):
        decision = adapt_revisit_pointgoal(
            mode="navdp_front_support_v1",
            router_active=True,
            pointgoal=[0.0, 0.0],
        )
        self.assertFalse(decision.takeover)
        self.assertEqual(decision.reason, "zero_pointgoal")

    def test_audit_is_json_compatible_and_explicit(self):
        audit = adapt_revisit_pointgoal(
            mode="verified_bearing_v1",
            router_active=True,
            pointgoal=[0.0, -7.0],
            source="sift_ransac_geometry",
        ).audit_dict()
        self.assertEqual(audit["revisit_adapter_schema_version"], 2)
        self.assertEqual(audit["revisit_adapter_source"],
                         "sift_ransac_geometry")
        self.assertEqual(audit["memory_bearing_unit"], [0.0, -1.0])
        self.assertEqual(audit["memory_controller_pointgoal"], [0.0, -2.5])

    def test_canonical_configuration_rejects_oracle_and_controller_swaps(self):
        validate_revisit_adapter_configuration(
            mode="verified_bearing_v1",
            server_backend="hybrid_pose",
            revisit_controller="navdp_mixed",
            router_is_automatic_geometry=True,
        )
        invalid = (
            ("navdp", "navdp_mixed", True),
            ("hybrid_pose", "xnavdp_point", True),
            ("hybrid_pose", "navdp_mixed", False),
        )
        for backend, controller, automatic in invalid:
            with self.subTest(backend=backend, controller=controller):
                with self.assertRaises(ValueError):
                    validate_revisit_adapter_configuration(
                        mode="verified_bearing_v1",
                        server_backend=backend,
                        revisit_controller=controller,
                        router_is_automatic_geometry=automatic,
                    )

    def test_front_support_configuration_allows_known_phase_route_only_with_mixed(self):
        validate_revisit_adapter_configuration(
            mode="navdp_front_support_v1",
            server_backend="hybrid_pose",
            revisit_controller="navdp_mixed",
            router_is_automatic_geometry=False,
        )
        with self.assertRaises(ValueError):
            validate_revisit_adapter_configuration(
                mode="navdp_front_support_v1",
                server_backend="hybrid_pose",
                revisit_controller="navdp_point",
                router_is_automatic_geometry=False,
            )

    def test_raw_fixed_bearing_allows_role_free_phase_ablation_only_with_mixed(self):
        validate_revisit_adapter_configuration(
            mode="raw_fixed_bearing_v1",
            server_backend="hybrid_pose",
            revisit_controller="navdp_mixed",
            router_is_automatic_geometry=False,
        )
        with self.assertRaises(ValueError):
            validate_revisit_adapter_configuration(
                mode="raw_fixed_bearing_v1",
                server_backend="navdp",
                revisit_controller="navdp_mixed",
                router_is_automatic_geometry=False,
            )

    def test_certified_router_uses_the_same_verified_bearing_contract(self):
        validate_revisit_adapter_configuration(
            mode="verified_bearing_v1",
            server_backend="hybrid_pose",
            revisit_controller="navdp_mixed",
            router_is_automatic_geometry=True,
            router_is_certified_relocalization=True,
        )

    def test_pi3x_scale_free_direction_uses_verified_bearing_contract(self):
        decision = adapt_revisit_pointgoal(
            mode="verified_bearing_v1",
            router_active=True,
            pointgoal=[3.0, -4.0],
            source="pi3x_b16_learned_spatial_proof_v1",
            pointgoal_units="pi3x_current_camera_direction_only",
        )
        self.assertTrue(decision.takeover)
        self.assertEqual(decision.reason, "verified_scale_free_bearing")
        self.assertEqual(decision.controller_pointgoal, (1.5, -2.0))
        self.assertIsNone(decision.raw_distance_m)

    def test_metric_only_modes_fail_closed_on_scale_free_units(self):
        for mode in (
            "legacy_metric",
            "navdp_front_support_v1",
            "raw_fixed_bearing_v1",
        ):
            with self.subTest(mode=mode):
                decision = adapt_revisit_pointgoal(
                    mode=mode,
                    router_active=True,
                    pointgoal=[3.0, -4.0],
                    pointgoal_units="lingbot_raw_direction_only",
                )
                self.assertFalse(decision.takeover)
                self.assertEqual(decision.reason, "metric_units_required")


if __name__ == "__main__":
    unittest.main()
