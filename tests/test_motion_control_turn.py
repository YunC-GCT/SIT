import math
import os
import tempfile
import unittest
from unittest.mock import patch

from core import make_result, path_segment_payload, read_result, write_result
from motion_control import run_follower
from motion_control.src.ros2_interface import (
    MotionSafetyError,
    Ros2PathExecutor,
    compute_segment_turns,
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


def non_zero(messages):
    return [message for message in messages if message != (0.0, 0.0, 0.0)]


class MotionControlTurnTests(unittest.TestCase):
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
