import unittest
import numpy as np

from path_planner.src.planners.prm import PRMPlanner
from path_planner.src.planners.rrt import RRTPlanner
from path_planner.src.planners.rrt_adaptive import RRTAdaptivePlanner
from path_planner.src.planners.rrt_star import RRTStarPlanner
from path_planner.src.grid_utils import line_of_sight
from path_planner.src.planner_factory import validate_algorithm_parameters, PlannerParameterError
from path_planner.src.planner_runner import run_algorithm


class SamplingRegressions(unittest.TestCase):
    def test_basic_rrt_rejects_adaptive_parameters(self):
        with self.assertRaises(PlannerParameterError):
            validate_algorithm_parameters('rrt', {'adaptive_step': True})

    def test_basic_rrt_extension_matches_fixed_steer(self):
        planner = RRTPlanner(np.zeros((20, 20), dtype=np.uint8), step_size=5)
        for start, sample in [((0, 0), (19, 19)), ((3, 7), (18, 2)), ((0, 0), (1, 1))]:
            expected = tuple(int(round(value)) for value in planner._steer(start, sample))
            self.assertEqual(planner._extension(start, sample), expected)

    def test_strategy_parameter_validation(self):
        for algorithm, parameters in [('prm', {'retry_num_samples': -1}),
                ('rrt_adaptive', {'adaptive_step': 1}), ('rrt_star', {'optimize_iterations': -1}),
                ('rrt_star', {'search_time_s': float('nan')}),
                ('rrt_star', {'optimize_time_s': float('inf')})]:
            with self.subTest(parameters=parameters), self.assertRaises(PlannerParameterError):
                validate_algorithm_parameters(algorithm, parameters)

    def test_search_diagnostics_are_exposed_on_success_and_failure(self):
        result = run_algorithm(np.zeros((5, 5), dtype=np.uint8), (0, 0), (4, 4),
            'rrt_star', parameters={'goal_sample_rate': 1., 'optimize_iterations': 0}, seed=42)
        self.assertEqual(result['status'], 'success')
        self.assertIsNotNone(result['outputs']['search_diagnostics']['first_solution_s'])
        result = run_algorithm(np.zeros((10, 10), dtype=np.uint8), (0, 0), (9, 9),
            'prm', parameters={'num_samples': 1, 'max_edge_len': 1, 'retry_num_samples': 2}, seed=42)
        self.assertEqual(result['status'], 'failed')
        self.assertEqual(len(result['outputs']['search_diagnostics']['attempts']), 2)

    def test_prm_retry_is_reported_and_parameter_is_restored(self):
        planner = PRMPlanner(np.zeros((10, 10), dtype=np.uint8), num_samples=1,
                             retry_num_samples=10, max_edge_len=1, seed=42)
        self.assertEqual(planner.plan((0, 0), (9, 9)), [])
        self.assertEqual([a['requested_samples'] for a in planner.diagnostics['attempts']], [1, 10])
        self.assertEqual(planner.num_samples, 1)

    def test_adaptive_extension_shrinks_near_obstacles(self):
        grid = np.zeros((8, 8), dtype=np.uint8)
        grid[:, 3] = 1
        fixed = RRTPlanner(grid, step_size=5)
        adaptive = RRTAdaptivePlanner(grid, step_size=5)
        self.assertIsNone(fixed._extension((2, 1), (2, 7)))
        self.assertEqual(adaptive._extension((2, 1), (2, 7)), (2, 2))

    def test_rrt_star_returns_after_first_solution_when_optimization_disabled(self):
        planner = RRTStarPlanner(np.zeros((10, 10), dtype=np.uint8), max_iter=10000,
            goal_sample_rate=1.0, optimize_iterations=0, seed=42)
        path = planner.plan((0, 0), (9, 9))
        self.assertEqual(path[-1], (9, 9))
        self.assertLess(planner.diagnostics['iterations'], 10)
        self.assertEqual(planner.diagnostics['stop_reason'], 'optimize_iterations')
    def test_collision_checks_both_directions_of_undirected_edge(self):
        grid = np.zeros((3, 3), dtype=np.uint8)
        grid[0, 1] = 1
        self.assertFalse(line_of_sight(grid, 3, 3, (0, 0), (1, 2)))
        self.assertFalse(line_of_sight(grid, 3, 3, (1, 2), (0, 0)))

    def test_start_equal_goal(self):
        for cls in (PRMPlanner, RRTPlanner, RRTStarPlanner):
            self.assertEqual(cls(np.zeros((3, 3), dtype=np.uint8), seed=42).plan((1, 1), (1, 1)), [(1, 1)])

    def test_prm_samples_unique_free_points_and_excludes_endpoints(self):
        grid = np.ones((20, 20), dtype=np.uint8)
        grid[0, :12] = 0
        points = PRMPlanner(grid, num_samples=10, seed=42)._sample_free_points((0, 0), (0, 11))
        self.assertEqual(len(set(points)), 12)

    def test_prm_searches_past_blocked_nearest_neighbors(self):
        grid = np.zeros((20, 20), dtype=np.uint8)
        grid[:, 2] = 1
        points = [(2, 1), (12, 1), (2, 3), (12, 3)]
        planner = PRMPlanner(grid, k_neighbors=1, max_edge_len=15)
        roadmap, _ = planner._build_roadmap(points)
        self.assertIn(1, roadmap[0])
        self.assertEqual(len(roadmap[0]), len(set(roadmap[0])))

    def test_rrt_star_keeps_valid_nearest_parent_outside_radius(self):
        grid = np.zeros((6, 6), dtype=np.uint8)
        grid[1, 2] = 1
        planner = RRTStarPlanner(grid, step_size=3, search_radius=2.5)
        parent, cost = planner._choose_parent([((0, 0), 0., -1), ((2, 2), 4., 0)], (0, 3), [1], 0)
        self.assertEqual((parent, cost), (0, 3.))

    def test_rrt_star_rewire_propagates_descendant_cost(self):
        planner = RRTStarPlanner(np.zeros((6, 6), dtype=np.uint8))
        nodes = [((0, 0), 0., -1), ((4, 0), 4., 0), ((4, 4), 8., 1),
                 ((0, 4), 12., 2), ((0, 5), 13., 3), ((0, 2), 2., 0)]
        planner._rewire(nodes, 5, [3])
        self.assertEqual(nodes[4][1], 5.)

    def test_repeated_samples_do_not_add_duplicate_tree_nodes(self):
        for cls in (RRTPlanner, RRTStarPlanner):
            planner = cls(np.zeros((10, 10), dtype=np.uint8), max_iter=5, goal_sample_rate=0)
            planner._sample_free = lambda: (0, 0)
            original = planner._nearest_node
            sizes = []
            def capture(nodes, point):
                sizes.append(len(nodes))
                return original(nodes, point)
            planner._nearest_node = capture
            self.assertEqual(planner.plan((0, 0), (9, 9)), [])
            self.assertEqual(sizes, [1] * 5)
