"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.model_controller import (
  SmartCruiseControlModel, DECEL_ON, MARGIN_ON, RELEASE_HOLD)


def model_sm(v_planned: float, a_planned: float):
  """A modelV2 the way plannerd sees it: a READER, not a builder.

  That distinction is the whole point of this helper. A builder tolerates slicing, a reader
  does not - indexing a capnp list reader with a slice raises "TypeError: an integer is
  required". Testing against a builder would have let the bug that took plannerd down on
  2026-09-29 sail straight through.

  The builder is returned alongside because the reader only borrows its segments: drop it and
  the reader dangles.
  """
  msg = messaging.new_message('modelV2')
  n = ModelConstants.IDX_N
  msg.modelV2.velocity.x = [v_planned] * n
  msg.modelV2.acceleration.x = [a_planned] * n
  return {'modelV2': msg.as_reader().modelV2}, msg


class TestSmartCruiseControlModel:

  def setup_method(self):
    self.params = Params()
    self.params.put_bool("SmartCruiseControlVision", True, block=True)
    self.scc = SmartCruiseControlModel()

  def run(self, sm_msg, v_ego, long_enabled=True, long_override=False, frames=1):
    sm, _keepalive = sm_msg
    for _ in range(frames):
      self.scc.update(sm, long_enabled, long_override, v_ego, 0.0, v_ego)
    return self.scc.output_v_target

  def test_initial_state(self):
    assert not self.scc.is_active
    assert self.scc.output_v_target == V_CRUISE_UNSET

  def test_runs_on_a_real_message(self):
    """Regression: the controller used to slice the capnp list directly and raise TypeError,
    which took plannerd down and left the car unable to engage."""
    self.run(model_sm(20.0, 0.0), 20.0)

  def test_idle_when_no_deceleration_is_planned(self):
    assert self.run(model_sm(20.0, 0.0), 20.0) == V_CRUISE_UNSET
    assert not self.scc.is_active

  def test_engages_on_a_planned_slowdown(self):
    v_ego = 25.0
    out = self.run(model_sm(v_ego - MARGIN_ON - 2.0, DECEL_ON - 0.2), v_ego)
    assert self.scc.is_active
    assert out < v_ego, "target must sit below the current speed, never above it"

  def test_stays_idle_when_the_drop_is_too_small(self):
    v_ego = 25.0
    assert self.run(model_sm(v_ego - 0.5, DECEL_ON - 0.2), v_ego) == V_CRUISE_UNSET

  def test_never_below_the_floor(self):
    v_ego = MIN_V + 2.0
    out = self.run(model_sm(0.0, DECEL_ON - 1.0), v_ego, frames=200)
    assert out == V_CRUISE_UNSET or out >= MIN_V

  def test_release_is_held(self):
    v_ego = 25.0
    self.run(model_sm(v_ego - MARGIN_ON - 2.0, DECEL_ON - 0.2), v_ego)
    assert self.scc.is_active
    # One frame of the release condition must not drop it.
    self.run(model_sm(v_ego, 0.0), v_ego)
    assert self.scc.is_active, f"must hold for {RELEASE_HOLD} s before releasing"

  def test_idle_while_the_driver_overrides(self):
    v_ego = 25.0
    assert self.run(model_sm(v_ego - MARGIN_ON - 2.0, DECEL_ON - 0.2), v_ego, long_override=True) == V_CRUISE_UNSET
    assert self.run(model_sm(v_ego - MARGIN_ON - 2.0, DECEL_ON - 0.2), v_ego, long_enabled=False) == V_CRUISE_UNSET

  def test_system_disabled(self):
    self.params.put_bool("SmartCruiseControlVision", False, block=True)
    scc = SmartCruiseControlModel()
    v_ego = 25.0
    sm, _keepalive = model_sm(v_ego - MARGIN_ON - 2.0, DECEL_ON - 0.2)
    assert scc.update(sm, True, False, v_ego, 0.0, v_ego) is None
    assert scc.output_v_target == V_CRUISE_UNSET
