import unittest

from algorithm.maze_logic import (
    Action,
    GreenStopDemoController,
    RightHandMazeController,
    SensorFrame,
    Signal,
)


class MazeLogicTests(unittest.TestCase):
    def test_red_stops_until_explicit_green(self):
        controller = RightHandMazeController()

        first = controller.decide(SensorFrame(30, 10, 10, Signal.STOP))
        self.assertEqual(first.action, Action.STOP)
        self.assertEqual(first.state, "STOPPED_RED")

        held = controller.decide(SensorFrame(30, 10, 10, Signal.UNKNOWN))
        self.assertEqual(held.action, Action.STOP)
        self.assertEqual(held.reason, "waiting for explicit GO")

        released = controller.decide(SensorFrame(30, 10, 10, Signal.GO))
        self.assertEqual(released.action, Action.FORWARD)

    def test_right_hand_priority_at_front_wall(self):
        controller = RightHandMazeController()
        decision = controller.decide(SensorFrame(8, 8, 30, Signal.UNKNOWN))
        self.assertEqual(decision.action, Action.TURN_RIGHT)

    def test_dead_end_becomes_uturn(self):
        controller = RightHandMazeController()
        decision = controller.decide(SensorFrame(8, 8, 8, Signal.UNKNOWN))
        self.assertEqual(decision.action, Action.UTURN)

    def test_right_opening_is_taken_even_when_front_open(self):
        controller = RightHandMazeController()
        decision = controller.decide(SensorFrame(30, 10, 30, Signal.UNKNOWN))
        self.assertEqual(decision.action, Action.TURN_RIGHT)

    def test_centering_pwm_steers_toward_more_space(self):
        controller = RightHandMazeController()
        decision = controller.decide(SensorFrame(30, 8, 14, Signal.UNKNOWN))
        self.assertEqual(decision.action, Action.FORWARD)
        self.assertGreater(decision.left_pwm, decision.right_pwm)

    def test_front_sensor_failure_stops(self):
        controller = RightHandMazeController()
        decision = controller.decide(SensorFrame(None, 10, 10, Signal.UNKNOWN))
        self.assertEqual(decision.action, Action.STOP)
        self.assertEqual(decision.state, "SENSOR_FAIL")


class DemoLogicTests(unittest.TestCase):
    def test_demo_uturn_until_green_then_latches_stop(self):
        controller = GreenStopDemoController()

        self.assertEqual(controller.decide(Signal.UNKNOWN).action, Action.UTURN)
        self.assertEqual(controller.decide(Signal.STOP).action, Action.UTURN)

        green = controller.decide(Signal.GO)
        self.assertEqual(green.action, Action.STOP)
        self.assertEqual(green.state, "DEMO_STOPPED_GREEN")

        held = controller.decide(Signal.UNKNOWN)
        self.assertEqual(held.action, Action.STOP)
        self.assertEqual(held.reason, "green already seen")


if __name__ == "__main__":
    unittest.main()
