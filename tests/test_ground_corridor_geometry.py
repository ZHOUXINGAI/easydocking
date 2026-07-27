from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from ground_corridor_geometry import (  # noqa: E402
    compute_ground_corridor_plan,
    dot,
    external_tangent_candidates,
    norm,
    select_direction_compatible_tangent,
    subtract,
)


class GroundCorridorGeometryTest(unittest.TestCase):
    CENTER = (0.0, 0.0)
    CARRIER = (-7.0, -6.0)
    RADIUS = 4.5

    def test_known_rover_tangencies_are_exact(self) -> None:
        for direction in ("ccw", "cw"):
            candidates = external_tangent_candidates(
                self.CENTER,
                self.RADIUS,
                self.CARRIER,
                direction,
            )
            for candidate in candidates:
                with self.subTest(direction=direction, point=candidate.point):
                    radius_vector = subtract(candidate.point, self.CENTER)
                    carrier_to_tangent = subtract(self.CARRIER, candidate.point)
                    self.assertAlmostEqual(norm(radius_vector), self.RADIUS, places=12)
                    self.assertAlmostEqual(
                        dot(carrier_to_tangent, radius_vector),
                        0.0,
                        places=12,
                    )
                    self.assertLess(abs(candidate.radius_residual_m), 1.0e-12)
                    self.assertLess(
                        abs(candidate.orthogonality_residual_m2),
                        1.0e-12,
                    )

    def test_direction_compatible_branch_has_recomputed_coordinates(self) -> None:
        selected = select_direction_compatible_tangent(
            self.CENTER,
            self.RADIUS,
            self.CARRIER,
            "ccw",
        )
        self.assertAlmostEqual(selected.point[0], 0.8883757492037598, places=12)
        self.assertAlmostEqual(selected.point[1], -4.411438374071053, places=12)
        self.assertAlmostEqual(
            selected.direction[0],
            0.9803196386824563,
            places=12,
        )
        self.assertAlmostEqual(
            selected.direction[1],
            0.1974168331563911,
            places=12,
        )
        self.assertAlmostEqual(selected.approach_alignment, 1.0, places=12)

    def test_vector_construction_matches_independent_acos_construction(self) -> None:
        candidates = external_tangent_candidates(
            self.CENTER,
            self.RADIUS,
            self.CARRIER,
            "ccw",
        )
        alpha = math.atan2(self.CARRIER[1], self.CARRIER[0])
        beta = math.acos(self.RADIUS / math.hypot(*self.CARRIER))
        angular_points = {
            (
                round(self.RADIUS * math.cos(alpha + beta), 12),
                round(self.RADIUS * math.sin(alpha + beta), 12),
            ),
            (
                round(self.RADIUS * math.cos(alpha - beta), 12),
                round(self.RADIUS * math.sin(alpha - beta), 12),
            ),
        }
        vector_points = {
            (round(candidate.point[0], 12), round(candidate.point[1], 12))
            for candidate in candidates
        }
        self.assertEqual(vector_points, angular_points)

    def test_legacy_asin_angle_is_not_a_tangent(self) -> None:
        alpha = math.atan2(self.CARRIER[1], self.CARRIER[0])
        legacy_beta = math.asin(self.RADIUS / math.hypot(*self.CARRIER))
        for sign in (1.0, -1.0):
            legacy_theta = alpha + sign * legacy_beta
            legacy_point = (
                self.RADIUS * math.cos(legacy_theta),
                self.RADIUS * math.sin(legacy_theta),
            )
            residual = dot(
                subtract(self.CARRIER, legacy_point),
                subtract(legacy_point, self.CENTER),
            )
            with self.subTest(sign=sign):
                self.assertGreater(abs(residual), 1.0)

    def test_cw_selects_the_other_direction_compatible_branch(self) -> None:
        selected = select_direction_compatible_tangent(
            self.CENTER,
            self.RADIUS,
            self.CARRIER,
            "cw",
        )
        self.assertAlmostEqual(selected.point[0], -4.223669866850818, places=12)
        self.assertAlmostEqual(selected.point[1], 1.5526148446592885, places=12)
        self.assertAlmostEqual(selected.approach_alignment, 1.0, places=12)

    def test_tangent_degeneracies_are_rejected(self) -> None:
        for carrier in ((0.0, 0.0), (4.5, 0.0), (1.0, 1.0)):
            with self.subTest(carrier=carrier):
                with self.assertRaises(ValueError):
                    external_tangent_candidates(
                        self.CENTER,
                        self.RADIUS,
                        carrier,
                        "ccw",
                    )
        with self.assertRaises(ValueError):
            external_tangent_candidates(self.CENTER, 0.0, self.CARRIER, "ccw")
        for carrier in ((math.nan, 0.0), (math.inf, 0.0)):
            with self.subTest(carrier=carrier):
                with self.assertRaisesRegex(ValueError, "must be finite"):
                    external_tangent_candidates(
                        self.CENTER,
                        self.RADIUS,
                        carrier,
                        "ccw",
                    )

    def test_plan_is_deterministic_and_approach_is_degenerate_arc(self) -> None:
        kwargs = dict(
            plan_id=11,
            sequence=7,
            frame_id="field_enu",
            origin_id=9,
            sender_monotonic_ms=1234,
            carrier_position=self.CARRIER,
            mini_phase_rad=math.radians(315.0),
            orbit_center=self.CENTER,
            orbit_radius_m=self.RADIUS,
            turn_direction="ccw",
            mini_speed_mps=0.9,
            mini_max_accel_mps2=0.5,
            carrier_max_speed_mps=0.7,
            carrier_max_accel_mps2=0.3,
        )
        first = compute_ground_corridor_plan(**kwargs)
        second = compute_ground_corridor_plan(**kwargs)
        self.assertEqual(first, second)
        self.assertEqual(first.carrier_approach.kind, "straight_tangent")
        self.assertEqual(first.carrier_approach.curvature_per_m, 0.0)
        self.assertAlmostEqual(first.carrier_approach.length_m, 8.04673846971554)
        self.assertLessEqual(
            first.carrier_approach.planned_speed_mps,
            first.carrier_max_speed_mps,
        )

    def test_cpp_uses_correct_formula_and_straight_tangent_marker(self) -> None:
        source = (
            REPO_ROOT
            / "src/easydocking_control/src/docking_controller.cpp"
        ).read_text(encoding="utf-8")
        start = source.index("void DockingController::computeCorridorPlan()")
        end = source.index("void DockingController::approachPhaseControl", start)
        block = source[start:end]
        self.assertIn("std::acos(R / d_oc)", block)
        self.assertNotIn("std::asin(R / d_oc)", block)
        self.assertIn("!std::isfinite(d_oc)", block)
        self.assertIn("!C.allFinite()", block)
        self.assertIn("const double r_arc = 0.0", block)
        self.assertIn("const double arc_len = (T - C).norm()", block)

    def test_protocol_keeps_runtime_code_and_coordination_transports_separate(
        self,
    ) -> None:
        protocol = (
            REPO_ROOT / "docs/ground_two_rover_offline_protocol.md"
        ).read_text(encoding="utf-8")
        normalized = " ".join(protocol.split())
        self.assertIn("LR24 Pair B is only the compact runtime link", normalized)
        self.assertIn("must not carry files", normalized)
        self.assertIn("Code and file deployment uses GitHub/SSH", normalized)
        self.assertIn("NATS", normalized)
        self.assertIn("never a vehicle runtime transport", normalized)
        self.assertIn(
            "`received_local_ms` is deliberately not transmitted",
            normalized,
        )


if __name__ == "__main__":
    unittest.main()
