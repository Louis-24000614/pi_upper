"""规则图 7 与真实拓扑、感知确认和地图写入的离线回归。"""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from road_follow.culvert import CulvertConfig, CulvertController, CulvertTarget, OdomHistory
from road_follow.culvert_map import CulvertRecords
from road_follow.obstacle_edge import OBSTACLE_DEFAULTS, filter_current_edge
from road_follow.placement_rules import placement_allowed
from topo_proto.graph import Edge, load_topology
from vision.obstacle.detect import Detection
import test_culvert_perception as perception_fixture
import test_culvert_runtime as runtime_fixture


ROOT = Path(__file__).resolve().parents[3]
BLUE = {
    "2_1__2_2", "2_3__2_4", "3_1__3_2", "3_3__3_4",
    "4_1__4_2", "4_3__4_4", "5_1__5_2", "5_3__5_4",
    "2_1__3_1", "3_1__4_1", "4_1__5_1",
    "2_2__3_2", "3_2__4_2", "4_2__5_2",
    "2_3__3_3", "3_3__4_3", "4_3__5_3",
    "2_4__3_4", "3_4__4_4", "4_4__5_4",
}
ORANGE = {"1_2__2_2", "1_3__2_3"}
TUNNELS = {"2_2__2_3", "3_2__3_3", "4_2__4_3", "5_2__5_3"}
START = {"0_0__0_J", "0_J__1_2", "0_J__1_3"}


def graph():
    return load_topology(ROOT / "config/nav_topology.yaml")


class PlacementRulesTest(unittest.TestCase):
    def test_real_topology_matches_all_twenty_blue_and_two_orange_segments(self):
        g = graph()
        self.assertEqual(set(g.edges), BLUE | ORANGE | TUNNELS | START)
        self.assertEqual({e.id for e in g.edges.values() if placement_allowed(e, "obstacle")}, BLUE)
        self.assertEqual({e.id for e in g.edges.values() if placement_allowed(e, "culvert")}, BLUE | ORANGE)

    def test_endpoints_are_used_in_both_directions_without_requiring_edge_id_format(self):
        for e in graph().edges.values():
            reversed_edge = replace(e, id="arbitrary_id", u=e.v, v=e.u)
            for kind in ("culvert", "obstacle"):
                self.assertEqual(placement_allowed(reversed_edge, kind), placement_allowed(e, kind))

    def test_unknown_edges_and_tunnel_flag_do_not_bypass_allowed_locations(self):
        for kind in ("culvert", "obstacle"):
            self.assertFalse(placement_allowed(None, kind))
            self.assertFalse(placement_allowed(Edge("unknown", "a", "b", 1), kind))
            self.assertFalse(placement_allowed(replace(graph().edges["4_3__5_3"], tunnel=True), kind))
            self.assertFalse(placement_allowed(replace(graph().edges["5_2__5_3"], tunnel=False), kind))


class CulvertPlacementTest(unittest.TestCase):
    def fixture(self, edge, reverse=False):
        f = perception_fixture.CulvertPerceptionTest()
        f.setUp()
        f.edge = replace(edge, length_m=1.2)
        source, destination = (edge.v, edge.u) if reverse else (edge.u, edge.v)
        f.history.reset((edge.id, source, destination))
        for i in range(8):
            f.history.add((edge.id, source, destination), i/10, .2+i*.005, i*.005, 0, 0)
        return f, {"from_node": source, "to_node": destination}

    def test_blue_and_orange_confirm_only_after_three_frames_in_both_directions(self):
        g = graph()
        for key in sorted(BLUE | ORANGE):
            for reverse in (False, True):
                with self.subTest(edge=key, reverse=reverse):
                    f, direction = self.fixture(g.edges[key], reverse)
                    self.assertEqual(f.observe(.7, **direction).reason, "tracking")
                    self.assertEqual(f.observe(.8, **direction).reason, "tracking")
                    result = f.observe(.9, **direction)
                    self.assertEqual(result.reason, "confirmed")
                    self.assertEqual(result.target.edge_id, key)
                    self.assertAlmostEqual(result.target.canonical_offset_m,
                        f.edge.length_m-result.target.target_s_m if reverse else result.target.target_s_m)

    def test_tunnels_and_start_reject_high_score_boxes_without_confirmation_votes(self):
        g = graph()
        for key in sorted(TUNNELS | START):
            for reverse in (False, True):
                with self.subTest(edge=key, reverse=reverse):
                    f, direction = self.fixture(g.edges[key], reverse)
                    for t in (.7, .8, .9, 1.0):
                        result = f.observe(t, **direction)
                        self.assertEqual(result.reason, "outside_placement_area")
                        self.assertIsNone(result.target)
                        self.assertIsNone(result.bbox)
                        self.assertEqual(len(f.perception.positions), 0)

    def test_target_beyond_allowed_current_edge_is_not_assigned_back_to_it(self):
        f, direction = self.fixture(graph().edges["2_1__2_2"])
        f.edge = replace(f.edge, length_m=.45)
        self.assertEqual(f.observe(.7, **direction).reason, "outside_current_edge")
        self.assertEqual(len(f.perception.positions), 0)

    def test_forbidden_edge_clears_previous_votes_even_without_direction_change(self):
        f, direction = self.fixture(graph().edges["4_3__5_3"])
        f.observe(.7, **direction)
        f.observe(.8, **direction)
        f.edge = replace(f.edge, tunnel=True)
        self.assertEqual(f.observe(.9, **direction).reason, "outside_placement_area")
        self.assertEqual(len(f.perception.positions), 0)
        f.edge = replace(f.edge, tunnel=False)
        result = f.observe(1.0, **direction)
        self.assertEqual(result.reason, "tracking")
        self.assertEqual(result.stable_frames, 1)

    def test_runtime_on_tunnel_does_not_discover_or_take_control(self):
        f = runtime_fixture.CulvertRuntimeTest()
        f.setUp()
        self.addCleanup(f.doCleanups)
        g = graph()
        key = "5_2__5_3"
        f.graph = f.agent.graph = f.runtime.records.graph = g
        f.agent.state.current_edge, f.agent.state.from_node, f.agent.state.to_node = key, "5_2", "5_3"
        for t in (.7, .8, .9, 1.0):
            result = f.update(t)
            self.assertFalse(result.owns)
            self.assertEqual(result.command, f.command)
        self.assertEqual(f.runtime.controller.phase, "idle")
        self.assertEqual(f.runtime.records.entries, {})
        self.assertEqual(f.sent, [])

    def test_unknown_target_on_tunnel_still_triggers_existing_stop_protection(self):
        f = runtime_fixture.CulvertRuntimeTest()
        f.setUp()
        self.addCleanup(f.doCleanups)
        f.graph = f.agent.graph = f.runtime.records.graph = graph()
        f.agent.state.current_edge = "5_2__5_3"
        f.agent.state.from_node, f.agent.state.to_node = "5_2", "5_3"
        for t in (.7, .8, .9):
            f.update(t, detections=[replace(f.box, class_id=9, label="unknown")])
        self.assertEqual(f.runtime.controller.phase, "fault")
        self.assertEqual(f.runtime.controller.fault_reason, "unknown_near_target")
        self.assertEqual(f.sent, ["stop"])
        self.assertEqual(f.runtime.records.entries, {})


class ObstaclePlacementTest(unittest.TestCase):
    def setUp(self):
        self.graph = graph()
        self.box = Detection(2, "fence", .9, 280, 80, 360, 300)
        self.ipm = NS(image_points_to_ground=lambda _: [[0, .35]])

    def filter(self, key, source, destination, progress=.2, **kwargs):
        return filter_current_edge([self.box], graph=self.graph,
            edge_key=(key, source, destination), progress_m=progress, ipm=self.ipm,
            image_shape=(480, 640), settings=OBSTACLE_DEFAULTS, **kwargs)

    def test_blue_current_edges_remain_eligible_in_both_directions(self):
        for key in sorted(BLUE):
            edge = self.graph.edges[key]
            for source, destination in ((edge.u, edge.v), (edge.v, edge.u)):
                with self.subTest(edge=key, source=source):
                    items, details = self.filter(key, source, destination)
                    self.assertEqual(items, [self.box])
                    self.assertEqual(details[0]["reason"], "current_edge")

    def test_orange_and_tunnels_cannot_confirm_or_leave_obstacle_markers(self):
        records = CulvertRecords(self.graph)
        for key in sorted(ORANGE | TUNNELS):
            edge = self.graph.edges[key]
            for source, destination in ((edge.u, edge.v), (edge.v, edge.u)):
                with self.subTest(edge=key, source=source):
                    items, details = self.filter(key, source, destination)
                    self.assertEqual(items, [])
                    self.assertEqual(details[0]["reason"], "outside_placement_area")
                    records.observe_obstacles(details, (280, 80, 360, 300))
        self.assertEqual(records.obstacles, {})
        self.assertEqual(self.graph.blocked, set())

    def test_allowed_car_edge_does_not_allow_target_projected_into_a_tunnel(self):
        items, details = self.filter("2_1__2_2", "2_1", "2_2", progress=.7)
        self.assertEqual(items, [])
        self.assertEqual(details[0]["target"]["edge_id"], "2_2__2_3")
        self.assertEqual(details[0]["reason"], "outside_placement_area")
        records = CulvertRecords(self.graph)
        records.observe_obstacles(details)
        self.assertEqual(records.obstacles, {})

    def test_orange_car_edge_can_observe_allowed_blue_target_ahead_without_blocking(self):
        items, details = self.filter("1_2__2_2", "1_2", "2_2", progress=.7)
        self.assertEqual(items, [])
        self.assertEqual(details[0]["target"]["edge_id"], "2_2__3_2")
        self.assertEqual(details[0]["reason"], "beyond_current_edge")
        records = CulvertRecords(self.graph)
        records.observe_obstacles(details)
        self.assertEqual(records.obstacles["2_2__3_2:2"]["status"], "observed")
        self.assertEqual(self.graph.blocked, set())

    def test_unknown_target_protection_can_keep_existing_geometry_on_tunnel(self):
        items, details = self.filter("5_2__5_3", "5_2", "5_3", placement_kind=None)
        self.assertEqual(items, [self.box])
        self.assertEqual(details[0]["reason"], "current_edge")


class PlacementMapTest(unittest.TestCase):
    def test_direct_culvert_map_write_rejects_forbidden_edges(self):
        g = graph()
        records = CulvertRecords(g)
        for key in sorted(TUNNELS | START):
            edge = g.edges[key]
            target = CulvertTarget(key, edge.u, edge.v, .2, .335, .47, .335, 0)
            with self.subTest(edge=key):
                self.assertFalse(records.discover(target))
        self.assertEqual(records.entries, {})

    def test_orange_and_previous_stop_edge_still_accept_culvert_map_records(self):
        g = graph()
        records = CulvertRecords(g)
        for key in sorted(ORANGE | {"4_3__5_3"}):
            edge = g.edges[key]
            records.discover(CulvertTarget(key, edge.u, edge.v, .2, .335, .47, .335, 0))
        self.assertEqual(set(records.entries), ORANGE | {"4_3__5_3"})

    def test_rejected_target_never_changes_controller_state_or_sends_commands(self):
        g = graph()
        records = CulvertRecords(g)
        commands, events = [], []
        config = CulvertConfig()
        controller = CulvertController(config, OdomHistory(config), send=commands.append,
            event=lambda *args, **kwargs: events.append((args, kwargs)), records=records)
        for key in sorted(TUNNELS | START):
            edge = g.edges[key]
            target = CulvertTarget(key, edge.u, edge.v, .2, .335, .47, .335, 0)
            with self.subTest(edge=key):
                self.assertFalse(controller.begin(target, 0, .1, .05))
                self.assertEqual(controller.phase, "idle")
                self.assertIsNone(controller.target)
        self.assertEqual(commands, [])
        self.assertEqual(events, [])
        self.assertEqual(records.entries, {})

    def test_mismatched_direction_cannot_write_a_culvert_on_an_allowed_edge(self):
        records = CulvertRecords(graph())
        target = CulvertTarget("4_3__5_3", "5_2", "5_3", .2, .335, .47, .335, 0)
        self.assertFalse(records.discover(target))
        self.assertEqual(records.entries, {})


if __name__ == "__main__":
    unittest.main()
