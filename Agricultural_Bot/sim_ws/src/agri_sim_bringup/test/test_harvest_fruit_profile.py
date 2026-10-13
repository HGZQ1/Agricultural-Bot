"""Offline fruit-layout contracts, without claiming IK or grasp validation."""

import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml


GENERATOR_PATH = (Path(__file__).parents[2] / "agri_greenhouse_worlds" / "scripts"
                  / "generate_tomato_field.py")
spec = importlib.util.spec_from_file_location("harvest_profile_generator", GENERATOR_PATH)
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


class HarvestFruitProfileTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.mission_path = Path(self.directory.name) / "mission.yaml"

    def args(self, seed=42, extra=()):
        return generator.parse_args([
            "--rows", "2", "--plants-per-row", "4", "--row-spacing", "3.0",
            "--plant-spacing", "0.7", "--origin-x", "-1.5", "--origin-y", "-1.0",
            "--plant-height", "2.4", "--plant-width", "1.0",
            "--plant-collision-width", "0.06", "--randomize-fruits",
            "--seed", str(seed), *extra,
        ])

    def mission(self, args, rotation=-math.pi / 2):
        tx, ty = 6.0, 0.0
        waypoints = []
        for plant in generator.plant_poses(args):
            # Keep one plant in each row untouched to verify random-stream isolation.
            if int(plant["id"].split("_")[1]) % args.plants_per_row == args.plants_per_row - 1:
                continue
            bx = -0.16 if plant["x"] < 0 else 0.16
            by, yaw = plant["y"] + 0.17998, math.pi / 2
            mx = tx + math.cos(rotation) * bx - math.sin(rotation) * by
            my = ty + math.sin(rotation) * bx + math.cos(rotation) * by
            waypoints.append({
                "id": "harvest_" + plant["id"], "task": "harvest", "enabled": True,
                "plant": {"id": plant["id"], "pose_world": plant},
                "base_pose": {"x": mx, "y": my, "yaw": yaw + rotation},
                "work_side": "left" if plant["x"] < 0 else "right",
            })
        data = {"alignment": {"world_to_map": [tx, ty, rotation]},
                "robot": {"arm_base_longitudinal_offset_m": -0.17998},
                "route": {"waypoints": waypoints}}
        self.mission_path.write_text(yaml.safe_dump(data), encoding="utf-8")
        return data

    def profile_args(self, seed=42, extra=()):
        base = self.args(seed, extra)
        self.mission(base)
        return self.args(seed, [*extra, "--harvest-mission", str(self.mission_path),
                                "--harvest-preserve-scan-max-z", "1.22"])

    def assert_clearances(self, args, fruits):
        geometry = generator.plant_geometry(args)
        half = geometry["collision_width"] / 2
        height = geometry["collision_height"]
        for index, fruit in enumerate(fruits):
            xyz = [fruit[key] for key in ("x", "y", "z")]
            self.assertGreaterEqual(fruit["z"] - fruit["radius"], args.fruit_min_clearance)
            for other in fruits[index + 1:]:
                self.assertGreaterEqual(
                    math.dist(xyz, [other[key] for key in ("x", "y", "z")]) + 1e-12,
                    fruit["radius"] + other["radius"] + args.fruit_min_clearance,
                )
            for plant in generator.plant_poses(args):
                dx, dy = fruit["x"] - plant["x"], fruit["y"] - plant["y"]
                lx = math.cos(plant["yaw"]) * dx + math.sin(plant["yaw"]) * dy
                ly = -math.sin(plant["yaw"]) * dx + math.cos(plant["yaw"]) * dy
                gap = math.hypot(max(abs(lx) - half, 0), max(abs(ly) - half, 0),
                                 max(-fruit["z"], fruit["z"] - height, 0))
                self.assertGreaterEqual(gap + 1e-12, fruit["radius"] + args.fruit_min_clearance)

    def test_legacy_random_stream_is_unchanged(self):
        args = generator.parse_args(["--randomize-fruits"])
        fruits = generator.sample_fruits(args)
        digest = hashlib.sha256(json.dumps(
            fruits, sort_keys=True, separators=(",", ":"),
        ).encode()).hexdigest()
        self.assertEqual(len(fruits), 597)
        self.assertEqual(digest, "f7ad5c9d5449a3afd283dfcf0947fbfb27b1e4e322c666e7bc7cf78f8fb78d75")

    def test_each_station_has_mature_nominal_candidates_across_seeds(self):
        for seed in (0, 1, 42, 103):
            with self.subTest(seed=seed):
                args = self.profile_args(seed)
                fruits = generator.sample_fruits(args)
                legacy = {fruit["id"]: fruit for fruit in generator.sample_fruits(self.args(seed))}
                profile = generator.harvest_profile_metadata(args, fruits)
                self.assertEqual(profile["status"], "distance_candidate_pending_ik")
                plants = {plant["id"]: plant for plant in generator.plant_poses(args)}
                stations = {station["plant_id"]: station for station in profile["stations"]}
                selected_count = near_count = 0
                for station in profile["stations"]:
                    self.assertGreaterEqual(len(station["ripe_distance_candidate_ids"]), 1)
                    members = [f for f in fruits if f["plant_id"] == station["plant_id"]]
                    selected_count += station["original_fruit_count"]
                    self.assertEqual(len(station["distance_candidate_ids"]),
                                     max(1, int(station["original_fruit_count"] * args.harvest_profile_ratio + 0.5)))
                    self.assertEqual(station["total_fruit_count"], len(members))
                    self.assertAlmostEqual(station["effective_near_fraction"],
                                           station["near_fruit_count"] / len(members))
                for fruit in fruits:
                    candidate = fruit.get("harvest_profile")
                    if candidate is None:
                        original = legacy[fruit["id"]]
                        if fruit["plant_id"] not in stations:
                            self.assertEqual(fruit, original)
                        else:
                            for key in ("id", "plant_id", "x", "y", "z", "radius"):
                                self.assertEqual(fruit[key], original[key])
                            self.assertFalse(fruit["ripe"])
                        continue
                    near_count += 1
                    plant, station = plants[fruit["plant_id"]], stations[fruit["plant_id"]]
                    self.assertEqual(candidate["status"], "distance_candidate_pending_ik")
                    self.assertEqual(candidate["ik_status"], "pending")
                    angle = math.atan2(fruit["y"] - plant["y"], fruit["x"] - plant["x"])
                    delta = math.atan2(math.sin(angle - station["near_side_sector_center_world_rad"]),
                                       math.cos(angle - station["near_side_sector_center_world_rad"]))
                    self.assertLessEqual(abs(delta), args.harvest_sector_half_angle + 1e-12)
                    self.assertLessEqual(candidate["grasp_flange_worst_distance_m"], args.harvest_flange_radius)
                    self.assertLessEqual(candidate["pregrasp_flange_worst_distance_m"], args.harvest_flange_radius)
                    self.assertEqual(candidate["nominal_approach_unit_world"][2], 0)
                    self.assertGreater(fruit["z"] - fruit["radius"], args.harvest_preserve_scan_max_z)
                    self.assertLessEqual(2 * fruit["radius"], args.harvest_diameter_max)
                    self.assertGreaterEqual(fruit["z"], args.harvest_height_min)
                    self.assertLessEqual(fruit["z"], args.harvest_height_max)
                self.assertGreaterEqual(near_count / selected_count, 0.75)
                self.assert_clearances(args, fruits)

    def test_nonidentity_alignment_respects_both_work_sides(self):
        args = self.profile_args()
        stations = generator.load_harvest_stations(args)
        self.assertEqual({station["work_side"] for station in stations}, {"left", "right"})
        for station in stations:
            self.assertAlmostEqual(station["nominal_base_pose_world"]["yaw"], math.pi / 2)
            sign = -1 if station["work_side"] == "left" else 1
            self.assertAlmostEqual(station["nominal_base_pose_world"]["x"], sign * 0.16)
            self.assertAlmostEqual(station["nominal_arm_reference_world_xyz_m"][0], sign * 0.16)
            self.assertAlmostEqual(station["parking_horizontal_allowance_m"],
                                   0.1 + 2 * 0.17998 * math.sin(0.08726646 / 2))

    def test_budget_includes_adverse_shoulder_and_parking_errors(self):
        args = self.profile_args()
        station = {"waypoint_id": "test", "work_side": "left",
                   "nominal_arm_reference_world_xyz_m": [0.0, 0.0, args.harvest_arm_reference_height],
                   "shoulder_horizontal_uncertainty_radius_m": 0.08,
                   "parking_horizontal_allowance_m": 0.11570123420857471}
        fruit = (0.68, 0.0, 1.33)
        candidate = generator.harvest_distance_candidate(args, station, fruit)
        self.assertIsNotNone(candidate)
        # The adverse shoulder and parked arm axis sit away from the target.
        worst_reference = [-0.08 - station["parking_horizontal_allowance_m"], 0,
                           args.harvest_arm_reference_height]
        self.assertAlmostEqual(candidate["grasp_flange_worst_distance_m"], math.dist(
            [0.68 - 0.16, 0, 1.33], worst_reference,
        ))
        self.assertAlmostEqual(candidate["pregrasp_flange_worst_distance_m"], math.dist(
            [0.68 - 0.16 - 0.08, 0, 1.33], worst_reference,
        ))
        # This would satisfy a nominal-only .70m allowance, but the complete
        # bound rejects it, preventing the former favorable-shoulder shortcut.
        args.harvest_flange_radius = 0.70
        self.assertIsNone(generator.harvest_distance_candidate(args, station, fruit))

    def test_protected_scan_spheres_are_retained_and_new_ids_are_added(self):
        common = ["--fruit-height-min", "1.0", "--fruit-height-max", "1.1",
                  "--fruit-count-min", "4", "--fruit-count-max", "4"]
        args = self.profile_args(extra=common)
        baseline = generator.sample_fruits(self.args(extra=common))
        fruits = generator.sample_fruits(args)
        by_id = {fruit["id"]: fruit for fruit in fruits}
        stations = generator.harvest_profile_metadata(args, fruits)["stations"]
        selected = {station["plant_id"] for station in stations}
        for original in baseline:
            preserved = by_id[original["id"]]
            for key in ("id", "plant_id", "x", "y", "z", "radius"):
                self.assertEqual(preserved[key], original[key])
            if original["plant_id"] in selected:
                self.assertFalse(preserved["ripe"])
                self.assertTrue(preserved["harvest_background"]["scan_geometry_preserved"])
            else:
                self.assertEqual(preserved, original)
        for station in stations:
            self.assertEqual(station["protected_fruit_count"], 4)
            self.assertEqual(station["near_fruit_count"], 3)
            self.assertEqual(station["total_fruit_count"], 7)
            self.assertTrue(all(int(fruit_id.rsplit("_", 1)[1]) >= 4
                                for fruit_id in station["distance_candidate_ids"]))
        self.assertEqual(len(fruits), len(baseline) + 6 * 3)
        self.assert_clearances(args, fruits)

    def test_zero_ratio_still_guarantees_one_mature_candidate(self):
        args = self.profile_args(extra=["--harvest-profile-ratio", "0", "--ripe-ratio", "0"])
        fruits = generator.sample_fruits(args)
        stations = generator.harvest_profile_metadata(args, fruits)["stations"]
        for station in stations:
            self.assertEqual(len(station["distance_candidate_ids"]), 1)
            self.assertEqual(len(station["ripe_distance_candidate_ids"]), 1)

    def test_zero_legacy_fruit_count_still_creates_one_mature_candidate_per_station(self):
        args = self.profile_args(extra=["--fruit-count-min", "0", "--fruit-count-max", "0"])
        fruits = generator.sample_fruits(args)
        stations = generator.harvest_profile_metadata(args, fruits)["stations"]
        self.assertEqual(len(fruits), len(stations))
        self.assertTrue(all(fruit["ripe"] for fruit in fruits))
        self.assertTrue(all(fruit["id"].endswith("_fruit_0") for fruit in fruits))
        for station in stations:
            self.assertEqual(station["original_fruit_count"], 0)
            self.assertEqual(station["near_fruit_count"], 1)
            self.assertEqual(len(station["ripe_distance_candidate_ids"]), 1)

    def test_infeasible_nominal_budget_is_rejected_without_partial_layout(self):
        args = self.profile_args(extra=["--harvest-flange-radius", "0.10"])
        original_attempts = generator.FRUIT_PLACEMENT_ATTEMPTS
        generator.FRUIT_PLACEMENT_ATTEMPTS = 5
        self.addCleanup(setattr, generator, "FRUIT_PLACEMENT_ATTEMPTS", original_attempts)
        with self.assertRaisesRegex(ValueError, "unable to place harvest distance candidates"):
            generator.sample_fruits(args)

    def test_stale_mission_or_incorrect_side_is_rejected(self):
        args = self.profile_args()
        mission = yaml.safe_load(self.mission_path.read_text())
        mission["route"]["waypoints"][0]["plant"]["pose_world"]["x"] -= 1
        self.mission_path.write_text(yaml.safe_dump(mission))
        with self.assertRaisesRegex(ValueError, "does not match generated field"):
            generator.load_harvest_stations(args)
        mission = self.mission(args)
        mission["route"]["waypoints"][0]["work_side"] = "right"
        self.mission_path.write_text(yaml.safe_dump(mission))
        with self.assertRaisesRegex(ValueError, "work_side conflicts"):
            generator.load_harvest_stations(args)

    def test_cli_emits_sdf_and_explicit_pending_validation_metadata(self):
        args = self.profile_args()
        output, metadata = Path(self.directory.name) / "field.sdf", Path(self.directory.name) / "field.json"
        arguments = []
        for name, value in vars(args).items():
            if name in ("output", "metadata") or value is None or value is False:
                continue
            arguments.append("--" + name.replace("_", "-"))
            if value is not True:
                arguments.append(str(value))
        result = subprocess.run([
            sys.executable, str(GENERATOR_PATH), *arguments,
            "--output", str(output), "--metadata", str(metadata),
        ], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(metadata.read_text())
        self.assertEqual(data["schema_version"], 2)
        self.assertEqual(data["harvest_profile"]["usage"], "offline_scene_generation_only")
        self.assertEqual(len(data["harvest_profile"]["stations"]), 6)
        models = {model.get("name"): model for model in ET.parse(output).getroot().find("world").findall("model")}
        for fruit in data["fruits"]:
            model = models[fruit["id"]]
            self.assertEqual([float(value) for value in model.findtext("pose").split()[:3]],
                             [fruit[key] for key in ("x", "y", "z")])
            self.assertEqual(float(model.findtext("link/collision/geometry/sphere/radius")), fruit["radius"])
            support = model.find("link/visual[@name='harvest_support_visual']")
            if "harvest_profile" in fruit:
                self.assertIsNotNone(support)
                self.assertEqual(float(support.findtext("geometry/cylinder/radius")), 0.004)
                self.assertEqual(len(model.findall("link/collision")), 1)
                # Stem and support share the fruit center's height; its full
                # cylinder lies above the retained scan band.
                self.assertGreater(fruit["z"] - 0.004, args.harvest_preserve_scan_max_z)
                self.assertEqual(float(support.findtext("pose").split()[2]), 0)
            else:
                self.assertIsNone(support)


if __name__ == "__main__":
    unittest.main()
