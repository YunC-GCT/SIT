import math
import os
import tempfile
import unittest
from unittest.mock import patch

from core import make_result, path_segment_payload, read_result, write_result
from motion_control import run_follower
from motion_control.src.ros2_interface import (
    CmdVelPreflight,
    ExecutionReport,
    MotionSafetyError,
    Ros2PathExecutor,
    compute_segment_turns,
    compute_turn_angles,
    delta_to_dir,
    estimate_runtime,
    heading_degrees,
    is_zero_twist,
    make_twist,
    normalize_angle_deg,
    split_turn_angle_deg,
)


class FakeEndpointInfo:
    def __init__(self, node_name, node_namespace="/"):
        self.node_name = node_name
        self.node_namespace = node_namespace


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def warn(self, message):
        self.messages.append(("warn", message))


class FakeNode:
    def __init__(self, subscribers=None, publishers=None, fail_graph_query=False):
        self.subscribers = subscribers or []
        self.publishers = publishers or []
        self.fail_graph_query = fail_graph_query
        self.logger = FakeLogger()
        self.destroy_count = 0

    def get_subscriptions_info_by_topic(self, topic):
        if self.fail_graph_query:
            raise RuntimeError("graph unavailable")
        self.last_subscription_topic = topic
        return self.subscribers

    def get_publishers_info_by_topic(self, topic):
        if self.fail_graph_query:
            raise RuntimeError("graph unavailable")
        self.last_publisher_topic = topic
        return self.publishers

    def get_namespace(self):
        return "/"

    def get_name(self):
        return "path_follower"

    def get_logger(self):
        return self.logger

    def destroy_node(self):
        self.destroyed = True
        self.destroy_count += 1


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, twist):
        self.messages.append(
            (
                float(twist.linear.x),
                float(twist.linear.y),
                float(twist.angular.z),
            )
        )


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class ZeroFailingPublisher(FakePublisher):
    def publish(self, twist):
        raise RuntimeError("publish failed")


def non_zero(messages):
    return [message for message in messages if message != (0.0, 0.0, 0.0)]


class MotionControlTurnTests(unittest.TestCase):
    def test_geometry_helpers_cover_cardinal_and_angle_mapping(self):
        self.assertEqual(delta_to_dir(-1, 0), 0)
        self.assertEqual(delta_to_dir(0, 1), 1)
        self.assertEqual(delta_to_dir(1, 0), 2)
        self.assertEqual(delta_to_dir(0, -1), 3)
        self.assertEqual(delta_to_dir(1, 1), 0)
        self.assertEqual(
            compute_turn_angles([1, 2, 3, 0], "north"),
            [90, 90, 90, 90],
        )
        self.assertEqual(heading_degrees("east"), 90.0)
        self.assertEqual(heading_degrees(370), 10.0)
        self.assertEqual(normalize_angle_deg(180), 180.0)
        self.assertEqual(normalize_angle_deg(-180), 180.0)
        self.assertEqual(normalize_angle_deg(540), 180.0)
        self.assertAlmostEqual(
            estimate_runtime([90.0, -90.0], 0.3, math.pi / 2.0, 0.3),
            4.0,
        )

    def test_twist_zero_helpers(self):
        self.assertTrue(is_zero_twist(make_twist()))
        self.assertFalse(is_zero_twist(make_twist(linear_x=0.1)))
        self.assertFalse(is_zero_twist(make_twist(angular_z=0.2)))

    def test_confirm_start_rejects_non_yes_and_non_tty(self):
        class Stream:
            def __init__(self, tty):
                self._tty = tty

            def isatty(self):
                return self._tty

        with patch("builtins.input", return_value="no"):
            with self.assertRaises(MotionSafetyError):
                run_follower._confirm_start(
                    assume_yes=False,
                    input_stream=Stream(True),
                )
        with self.assertRaises(MotionSafetyError):
            run_follower._confirm_start(
                assume_yes=False,
                input_stream=Stream(False),
            )

    def test_stop_only_connects_and_stops_without_writing_a_motion_result(self):
        class FakeExecutor:
            def __init__(self):
                self.stopped = False
                self.closed = False

            def setup_ros2(self):
                pass

            def stop_robot(self):
                self.stopped = True

            def close(self):
                self.closed = True

        executor = FakeExecutor()
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(
                run_follower,
                "_build_executor",
                return_value=executor,
            ):
                exit_code = run_follower.main(
                    [
                        "--stop-only",
                        "--run-root",
                        tmp,
                        "--run-id",
                        "stop-001",
                    ]
                )

        self.assertEqual(exit_code, 0)
        self.assertTrue(executor.stopped)
        self.assertTrue(executor.closed)

    def test_motion_cli_success_writes_execution_evidence(self):
        class FakeExecutor:
            def setup_ros2(self):
                pass

            def preflight_check(self, **_kwargs):
                return CmdVelPreflight(
                    topic="/cmd_vel",
                    subscribers=("/mecanum_chassis_node",),
                    publishers=("/path_follower",),
                    chassis_subscriber_seen=True,
                    other_publishers=(),
                    confirmed=True,
                    warnings=(),
                )

            def execute_segments(self, segments, require_preflight=False):
                return ExecutionReport(
                    segment_count=len(segments),
                    command_count=3,
                    open_loop=True,
                    completed=True,
                    stopped=True,
                )

            def close(self):
                pass

        with tempfile.TemporaryDirectory() as tmp:
            postprocess_path = os.path.join(tmp, "postprocess.json")
            write_result(
                postprocess_path,
                make_result(
                    "path_postprocess",
                    "post-001",
                    "success",
                    outputs={
                        "segments": [
                            path_segment_payload(
                                0,
                                start=(0, 0),
                                end=(0, 1),
                                direction_deg=90.0,
                                turn_angle_deg=90.0,
                                grid_distance=1.0,
                                distance_m=0.3,
                                estimated_time_s=2.0,
                            )
                        ]
                    },
                ),
            )
            with patch.object(
                run_follower,
                "_build_executor",
                return_value=FakeExecutor(),
            ):
                exit_code = run_follower.main(
                    [
                        "--postprocess-result",
                        postprocess_path,
                        "--yes",
                        "--run-root",
                        tmp,
                        "--run-id",
                        "motion-ok",
                    ]
                )

            result = read_result(
                os.path.join(tmp, "motion-ok", "motion_control", "result.json")
            )

        self.assertEqual(exit_code, 0)
        self.assertEqual(result["status"], "success")
        self.assertTrue(result["outputs"]["execution"]["completed"])
        self.assertTrue(result["outputs"]["execution"]["stopped"])

    def test_motion_cli_preflight_failure_is_structured(self):
        class FakeExecutor:
            def setup_ros2(self):
                pass

            def preflight_check(self, **_kwargs):
                return CmdVelPreflight(
                    topic="/cmd_vel",
                    subscribers=(),
                    publishers=("/gesture_control_node",),
                    chassis_subscriber_seen=False,
                    other_publishers=("/gesture_control_node",),
                    confirmed=False,
                    warnings=("other publisher",),
                )

            def close(self):
                pass

        with tempfile.TemporaryDirectory() as tmp:
            postprocess_path = os.path.join(tmp, "postprocess.json")
            write_result(
                postprocess_path,
                make_result(
                    "path_postprocess",
                    "post-002",
                    "success",
                    outputs={
                        "segments": [
                            path_segment_payload(
                                0,
                                start=(0, 0),
                                end=(0, 1),
                                direction_deg=90.0,
                                turn_angle_deg=90.0,
                                grid_distance=1.0,
                                distance_m=0.3,
                                estimated_time_s=2.0,
                            )
                        ]
                    },
                ),
            )
            with patch.object(
                run_follower,
                "_build_executor",
                return_value=FakeExecutor(),
            ):
                exit_code = run_follower.main(
                    [
                        "--postprocess-result",
                        postprocess_path,
                        "--yes",
                        "--run-root",
                        tmp,
                        "--run-id",
                        "motion-preflight",
                    ]
                )

            result = read_result(
                os.path.join(
                    tmp,
                    "motion-preflight",
                    "motion_control",
                    "result.json",
                )
            )

        self.assertEqual(exit_code, 2)
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["errors"][0]["code"], "MOTION_PREFLIGHT_FAILED")

    def make_executor(self, clock=None):
        clock = clock or FakeClock()
        return Ros2PathExecutor(
            turn_speed=math.pi / 2.0,
            move_speed=0.5,
            grid_size=0.3,
            timer_period=0.5,
            initial_heading="north",
            empirical_forward_speed_m_s=0.3,
            zero_transition_s=0.5,
            stop_publish_count=2,
            stop_publish_period_s=0.0,
            communication_buffer_s=0.0,
            sleep=clock.sleep,
        )

    def test_shortest_turns_use_map_north_and_split_180_degrees(self):
        segments = [
            {"index": 0, "direction_deg": 180.0, "distance_m": 0.3},
        ]

        self.assertEqual(compute_segment_turns(segments, initial_heading="north"), [180.0])
        self.assertEqual(split_turn_angle_deg(180.0), [90.0, 90.0])

        node = FakeNode(
            subscribers=[FakeEndpointInfo("mecanum_chassis_node")],
            publishers=[FakeEndpointInfo("path_follower")],
        )
        pub = FakePublisher()
        clock = FakeClock()
        report = self.make_executor(clock).execute_segments(segments, node=node, pub=pub)

        self.assertTrue(report.completed)
        self.assertTrue(report.stopped)
        self.assertEqual(clock.now, 5.0)
        self.assertEqual(node.last_subscription_topic, "/cmd_vel")
        self.assertEqual(node.last_publisher_topic, "/cmd_vel")
        self.assertIn((0.0, 0.0, -math.pi / 2.0), pub.messages)
        self.assertIn((0.5, 0.0, 0.0), pub.messages)
        self.assertEqual(pub.messages[-1], (0.0, 0.0, 0.0))

        moving = non_zero(pub.messages)
        self.assertEqual(
            moving,
            [
                (0.0, 0.0, -math.pi / 2.0),
                (0.0, 0.0, -math.pi / 2.0),
                (0.0, 0.0, -math.pi / 2.0),
                (0.0, 0.0, -math.pi / 2.0),
                (0.5, 0.0, 0.0),
                (0.5, 0.0, 0.0),
            ],
        )

    def test_total_forward_time_is_independent_of_segment_split(self):
        def forward_count(segments):
            node = FakeNode(
                subscribers=[FakeEndpointInfo("mecanum_chassis_node")],
                publishers=[FakeEndpointInfo("path_follower")],
            )
            pub = FakePublisher()
            self.make_executor().execute_segments(segments, node=node, pub=pub)
            return sum(1 for message in pub.messages if message == (0.5, 0.0, 0.0))

        one_segment = [{"index": 0, "direction_deg": 0.0, "distance_m": 3.0}]
        ten_segments = [
            {"index": index, "direction_deg": 0.0, "distance_m": 0.3}
            for index in range(10)
        ]

        self.assertEqual(forward_count(one_segment), forward_count(ten_segments))
        self.assertEqual(forward_count(one_segment), 20)

    def test_forward_rounding_is_carried_across_segments(self):
        def forward_count(segments):
            node = FakeNode(
                subscribers=[FakeEndpointInfo("mecanum_chassis_node")],
                publishers=[FakeEndpointInfo("path_follower")],
            )
            pub = FakePublisher()
            self.make_executor().execute_segments(segments, node=node, pub=pub)
            return sum(1 for message in pub.messages if message == (0.5, 0.0, 0.0))

        seven_segments = [
            {"index": index, "direction_deg": 0.0, "distance_m": 0.42}
            for index in range(7)
        ]
        three_segments = [
            {"index": index, "direction_deg": 0.0, "distance_m": 0.98}
            for index in range(3)
        ]

        self.assertEqual(forward_count(seven_segments), forward_count(three_segments))
        self.assertEqual(forward_count(seven_segments), 20)

    def test_stop_publish_failure_is_not_reported_as_stopped(self):
        node = FakeNode(
            subscribers=[FakeEndpointInfo("mecanum_chassis_node")],
            publishers=[FakeEndpointInfo("path_follower")],
        )
        executor = self.make_executor()

        with self.assertRaises(RuntimeError):
            executor.execute_segments(
                [{"index": 0, "direction_deg": 0.0, "distance_m": 0.3}],
                node=node,
                pub=ZeroFailingPublisher(),
            )

        self.assertFalse(executor._stopped)

    def test_preflight_blocks_other_cmd_vel_publishers_before_motion(self):
        node = FakeNode(
            subscribers=[FakeEndpointInfo("mecanum_chassis_node")],
            publishers=[
                FakeEndpointInfo("path_follower"),
                FakeEndpointInfo("gesture_control_node"),
            ],
        )
        pub = FakePublisher()

        with self.assertRaises(MotionSafetyError):
            self.make_executor().execute_segments(
                [{"index": 0, "direction_deg": 90.0, "distance_m": 0.3}],
                node=node,
                pub=pub,
            )

        self.assertTrue(pub.messages)
        self.assertTrue(all(message == (0.0, 0.0, 0.0) for message in pub.messages))

    def test_ros_graph_query_failure_blocks_motion(self):
        node = FakeNode(
            subscribers=[FakeEndpointInfo("mecanum_chassis_node")],
            publishers=[FakeEndpointInfo("path_follower")],
            fail_graph_query=True,
        )
        pub = FakePublisher()

        with self.assertRaises(MotionSafetyError):
            self.make_executor().execute_segments(
                [{"index": 0, "direction_deg": 90.0, "distance_m": 0.3}],
                node=node,
                pub=pub,
            )

        self.assertTrue(pub.messages)
        self.assertTrue(all(message == (0.0, 0.0, 0.0) for message in pub.messages))

    def test_missing_chassis_subscriber_blocks_motion(self):
        node = FakeNode(
            subscribers=[],
            publishers=[FakeEndpointInfo("path_follower")],
        )
        pub = FakePublisher()

        with self.assertRaises(MotionSafetyError):
            self.make_executor().execute_segments(
                [{"index": 0, "direction_deg": 90.0, "distance_m": 0.3}],
                node=node,
                pub=pub,
            )

        self.assertTrue(all(message == (0.0, 0.0, 0.0) for message in pub.messages))

    def test_repeated_close_keeps_publishing_zero_and_shutdown_is_idempotent(self):
        node = FakeNode(
            subscribers=[FakeEndpointInfo("mecanum_chassis_node")],
            publishers=[FakeEndpointInfo("path_follower")],
        )
        pub = FakePublisher()
        executor = self.make_executor()

        executor.execute_segments(
            [{"index": 0, "direction_deg": 0.0, "distance_m": 0.3}],
            node=node,
            pub=pub,
        )
        before = len(pub.messages)
        executor.close()
        executor.close()

        self.assertGreater(len(pub.messages), before)
        self.assertEqual(node.destroy_count, 1)
        self.assertTrue(all(message == (0.0, 0.0, 0.0) for message in pub.messages[-4:]))

    def test_cli_writes_structured_failure_when_ros2_startup_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            postprocess_path = os.path.join(tmp, "postprocess.json")
            write_result(
                postprocess_path,
                make_result(
                    "path_postprocess",
                    "post-001",
                    "success",
                    outputs={
                        "segments": [
                            path_segment_payload(
                                0,
                                start=(0, 0),
                                end=(0, 1),
                                direction_deg=90.0,
                                turn_angle_deg=90.0,
                                grid_distance=1.0,
                                distance_m=0.3,
                                estimated_time_s=2.0,
                            )
                        ]
                    },
                ),
            )

            with patch.object(
                run_follower.Ros2PathExecutor,
                "setup_ros2",
                side_effect=RuntimeError("ROS2 unavailable"),
            ):
                exit_code = run_follower.main(
                    [
                        "--postprocess-result",
                        postprocess_path,
                        "--yes",
                        "--run-root",
                        tmp,
                        "--run-id",
                        "motion-001",
                    ]
                )

            self.assertEqual(exit_code, 2)
            result = read_result(
                os.path.join(tmp, "motion-001", "motion_control", "result.json")
            )
            self.assertEqual(result["status"], "failed")
            self.assertEqual(result["errors"][0]["module"], "motion_control")
            self.assertEqual(result["errors"][0]["stage"], "execute")
            self.assertIn("ROS2 unavailable", result["errors"][0]["reason"])


if __name__ == "__main__":
    unittest.main()
