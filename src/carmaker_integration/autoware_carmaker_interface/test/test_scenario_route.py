"""Scenario route requests: file parsing and the AD API decision, without DDS."""

import math

import pytest
from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState

from autoware_carmaker_interface.scenario_route_node import load_route, next_action


def _mode(available=True, mode=OperationModeState.STOP):
    state = OperationModeState()
    state.is_autonomous_mode_available = available
    state.mode = mode
    return state


def test_goal_and_waypoints_are_read_from_the_scenario(tmp_path):
    path = tmp_path / "scenario.yaml"
    path.write_text("autoware:\n  goal: {x: 690.0, y: 0.5, z: 0.0, yaw: 1.5707963}\n"
                    "  waypoints:\n  - {x: 100.0, y: 2.0, yaw: 0.0}\n")
    goal, waypoints = load_route(path)
    assert goal.position.x == pytest.approx(690.0)
    assert 2.0 * math.atan2(goal.orientation.z, goal.orientation.w) == pytest.approx(math.pi / 2)
    assert len(waypoints) == 1 and waypoints[0].position.y == pytest.approx(2.0)
    path.write_text("autoware: {}\n")
    with pytest.raises(ValueError):
        load_route(path)


def test_route_is_set_only_after_localization_and_when_unset():
    assert next_action(False, RouteState.UNSET, None, True, False) is None
    assert next_action(True, RouteState.UNSET, None, False, False) == "set_route"
    assert next_action(True, RouteState.SET, _mode(), False, False) is None
    assert next_action(True, RouteState.ARRIVED, _mode(), True, False) is None


def test_engage_is_requested_once_when_available():
    assert next_action(True, RouteState.SET, _mode(), True, False) == "engage"
    assert next_action(True, RouteState.SET, _mode(), True, True) is None
    assert next_action(True, RouteState.SET, _mode(available=False), True, False) is None
    assert next_action(True, RouteState.SET, _mode(mode=OperationModeState.AUTONOMOUS), True, False) is None
