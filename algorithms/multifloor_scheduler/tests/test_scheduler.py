import unittest

from core import (
    BuildingMap, Edge, Plan, Robot, RouteCandidate, SchedulerConfig, Task,
    detect_conflicts, edge_time, generate_virtual, parse_payload, schedule,
)


class SchedulerTests(unittest.TestCase):
    def test_virtual_runs(self):
        payload = generate_virtual({"floors": 2, "width": 3, "height": 3, "robots": 3, "tasks": 3, "seed": 3})
        building, robots, tasks, config = parse_payload(payload)
        result = schedule(building, robots, tasks, config)
        self.assertEqual(result["metrics"]["assigned_count"], 3)

    def test_wheeled_does_not_use_stairs(self):
        payload = generate_virtual({"floors": 2, "width": 2, "height": 2, "elevators": 1, "stairs": 1, "robots": 1, "tasks": 1, "seed": 9})
        payload["robots"][0]["robot_type"] = "wheeled"
        building, robots, tasks, config = parse_payload(payload)
        result = schedule(building, robots, tasks, config)
        for assignment in result["assignments"]:
            self.assertNotIn("stair", [edge["kind"] for edge in assignment["edges"]])

    def test_task_type_must_match_exactly(self):
        payload = generate_virtual({"floors": 1, "width": 3, "height": 1, "elevators": 0, "stairs": 0, "robots": 1, "tasks": 1})
        payload["robots"][0]["task_type"] = "lab"
        payload["tasks"][0]["task_type"] = "pharmacy"
        building, robots, tasks, config = parse_payload(payload)
        result = schedule(building, robots, tasks, config)
        self.assertEqual(result["metrics"]["assigned_count"], 0)
        self.assertEqual(result["unassigned_tasks"], ["t0"])

    def test_default_speeds_and_vertical_times(self):
        payload = generate_virtual({"floors": 5, "width": 2, "height": 2, "elevators": 1, "stairs": 1, "robots": 1, "tasks": 1})
        building, robots, _, config = parse_payload(payload)
        self.assertEqual(config.default_speed_xy, 1.5)
        self.assertEqual(config.stair_speed, 0.5)
        self.assertEqual(robots[0].speed_xy, 1.5)
        self.assertAlmostEqual(edge_time(Edge("a", "b", 1, "stair"), robots[0], building, config), 7.0)
        direct_ride = next(
            edge for edges in building.adjacency.values() for edge in edges
            if edge.kind == "elevator"
            and abs(building.nodes[edge.src].floor - building.nodes[edge.dst].floor) == 4
        )
        self.assertAlmostEqual(edge_time(direct_ride, robots[0], building, config), 11.5)

    def test_simulation_generator_positions_and_counts(self):
        payload = generate_virtual({
            "floors": 4, "width": 4, "height": 3, "elevators": 2, "stairs": 2,
            "robots": 7, "robot_types": ["wheeled", "legged"],
            "tasks": 9, "task_types": ["medicine", "sample"], "seed": 13,
        })
        self.assertEqual(len(payload["map"]["nodes"]), 4 * 4 * 3)
        self.assertEqual(len(payload["robots"]), 7)
        self.assertEqual(len(payload["tasks"]), 9)
        self.assertEqual(sum(edge["kind"] == "elevator" for edge in payload["map"]["edges"]), 2 * 2 * 3)
        self.assertEqual(sum(edge["kind"] == "stair" for edge in payload["map"]["edges"]), 2 * 2 * 3)
        self.assertTrue(all({"x", "y", "z", "floor"}.issubset(node) for node in payload["map"]["nodes"]))
        self.assertEqual(len({(node["x"], node["y"]) for node in payload["map"]["nodes"] if node["node_type"] != "corridor"}), 4)

    def test_opposite_directions_on_bidirectional_floor_edges_do_not_conflict(self):
        building = BuildingMap.from_dict({
            "nodes": [
                {"id": "a", "floor": 0}, {"id": "b", "floor": 0},
            ],
            "edges": [
                {"src": "a", "dst": "b", "length": 1, "kind": "floor"},
                {"src": "b", "dst": "a", "length": 1, "kind": "floor"},
            ],
        })
        robot_a = Robot("r1", "wheeled", "a")
        robot_b = Robot("r2", "wheeled", "b")
        task_a = Task("t1", "a", "b")
        task_b = Task("t2", "b", "a")
        edge_ab = building.adjacency["a"][0]
        edge_ba = building.adjacency["b"][0]
        plans = [
            Plan(robot_a, task_a, [RouteCandidate(["a", "b"], [edge_ab], 1 / 1.5, "none")]),
            Plan(robot_b, task_b, [RouteCandidate(["b", "a"], [edge_ba], 1 / 1.5, "none")]),
        ]
        self.assertEqual(detect_conflicts(plans, building), [])

    def test_stairs_only_conflict_for_opposite_directions(self):
        building = BuildingMap.from_dict({
            "nodes": [
                {"id": "low_a", "floor": 0}, {"id": "low_b", "floor": 0},
                {"id": "high_a", "floor": 1}, {"id": "high_b", "floor": 1},
            ],
            "edges": [
                {"src": "low_a", "dst": "high_a", "length": 1, "kind": "stair", "resource": "s0"},
                {"src": "high_a", "dst": "low_a", "length": 1, "kind": "stair", "resource": "s0"},
                {"src": "low_b", "dst": "high_b", "length": 1, "kind": "stair", "resource": "s0"},
                {"src": "high_b", "dst": "low_b", "length": 1, "kind": "stair", "resource": "s0"},
            ],
            "resources": {"s0": 1},
        })
        up_a = building.adjacency["low_a"][0]
        up_b = building.adjacency["low_b"][0]
        down_a = building.adjacency["high_a"][0]
        same_direction = [
            Plan(Robot("r1", "legged", "low_a"), Task("t1", "low_a", "high_a"),
                 [RouteCandidate(["low_a", "high_a"], [up_a], 7.0, "stair")]),
            Plan(Robot("r2", "legged", "low_b"), Task("t2", "low_b", "high_b"),
                 [RouteCandidate(["low_b", "high_b"], [up_b], 7.0, "stair")]),
        ]
        self.assertEqual(detect_conflicts(same_direction, building), [])
        same_direction[1] = Plan(
            Robot("r2", "legged", "high_a"), Task("t2", "high_a", "low_a"),
            [RouteCandidate(["high_a", "low_a"], [down_a], 7.0, "stair")],
        )
        self.assertEqual(len(detect_conflicts(same_direction, building)), 1)

    def test_previous_batch_elevator_reservation_is_preserved(self):
        payload = generate_virtual({
            "floors": 2, "width": 2, "height": 1, "elevators": 1, "stairs": 0,
            "robots": 1, "tasks": 1,
        })
        payload["robots"][0]["task_type"] = "delivery"
        payload["tasks"][0].update({"task_type": "delivery", "pickup": "f1_x0_y0", "delivery": "f0_x0_y0"})
        building, robots, tasks, config = parse_payload(payload)
        first_batch = schedule(building, robots, tasks, config)
        previous_reservations = first_batch["resource_schedule"]
        second_batch = schedule(building, robots, tasks, config, previous_reservations)
        new_reservation = [
            entry for entry in second_batch["resource_schedule"]
            if entry["robot_id"] == robots[0].id
        ][-1]
        old_reservation = previous_reservations[0]
        self.assertGreaterEqual(new_reservation["start"], old_reservation["end"])

    def test_one_to_two_pickup_then_two_to_three_delivery_uses_same_elevator(self):
        payload = {
            "map": {
                "nodes": [
                    {"id": "f1_e1", "floor": 1},
                    {"id": "f2_e1", "floor": 2},
                    {"id": "f3_e1", "floor": 3},
                    {"id": "f2_pickup", "floor": 2},
                    {"id": "f3_delivery", "floor": 3},
                ],
                "edges": [
                    {"src": "f1_e1", "dst": "f2_e1", "length": 1, "kind": "elevator", "resource": "E1"},
                    {"src": "f2_e1", "dst": "f1_e1", "length": 1, "kind": "elevator", "resource": "E1"},
                    {"src": "f2_e1", "dst": "f3_e1", "length": 1, "kind": "elevator", "resource": "E1"},
                    {"src": "f3_e1", "dst": "f2_e1", "length": 1, "kind": "elevator", "resource": "E1"},
                    {"src": "f2_e1", "dst": "f2_pickup", "length": 1, "kind": "floor"},
                    {"src": "f2_pickup", "dst": "f2_e1", "length": 1, "kind": "floor"},
                    {"src": "f3_e1", "dst": "f3_delivery", "length": 1, "kind": "floor"},
                    {"src": "f3_delivery", "dst": "f3_e1", "length": 1, "kind": "floor"},
                ],
                "resources": {"E1": 1},
            },
            "robots": [{
                "id": "r1", "robot_type": "wheeled", "task_type": "medicine",
                "start_node": "f1_e1",
            }],
            "tasks": [{
                "id": "t1", "task_type": "medicine", "pickup": "f2_pickup",
                "delivery": "f3_delivery",
            }],
        }
        building, robots, tasks, config = parse_payload(payload)
        result = schedule(building, robots, tasks, config)
        self.assertEqual(result["metrics"]["assigned_count"], 1)
        elevator_windows = [
            item for item in result["resource_schedule"] if item["resource"] == "elevator:E1"
        ]
        self.assertEqual(len(elevator_windows), 2)
        self.assertLessEqual(elevator_windows[0]["end"], elevator_windows[1]["start"])


if __name__ == "__main__":
    unittest.main()
