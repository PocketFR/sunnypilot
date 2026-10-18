"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from opendbc.car import structs
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner

# (openpilotLongitudinalControl, pcmCruiseSpeed, ICBM is the actuator)
MODES = [
  (False, False, True),   # ICBM: the set speed ceiling is the only actuator
  (True, False, False),   # openpilot longitudinal, which is also the ESCC configuration
  (True, True, False),
  (False, True, False),   # stock ACC, no SCC candidate is even available
]

VISION, MODEL = 25.0, 15.0


def planner(openpilot_long: bool, pcm_cruise_speed: bool) -> LongitudinalPlanner:
  CP = structs.CarParams(openpilotLongitudinalControl=openpilot_long, carFingerprint="HYUNDAI_IONIQ_PHEV")
  CP_SP = structs.CarParamsSP(pcmCruiseSpeed=pcm_cruise_speed)
  return LongitudinalPlanner(CP, CP_SP)


class TestModelCandidateGating:
  """The model candidate must reach the min() in ICBM only - see scc_vision_v_target."""

  @pytest.mark.parametrize("openpilot_long, pcm_cruise_speed, icbm", MODES)
  def test_gate_matches_the_mode(self, openpilot_long, pcm_cruise_speed, icbm):
    assert planner(openpilot_long, pcm_cruise_speed).icbm_is_actuator == icbm

  @pytest.mark.parametrize("openpilot_long, pcm_cruise_speed, icbm", MODES)
  def test_model_only_counts_in_icbm(self, openpilot_long, pcm_cruise_speed, icbm):
    p = planner(openpilot_long, pcm_cruise_speed)
    p.scc.vision.output_v_target = VISION
    p.scc.model.output_v_target = MODEL
    assert p.scc_vision_v_target == (MODEL if icbm else VISION), \
      "with openpilot longitudinal the MPC already handles the lead this candidate reacts to"

  @pytest.mark.parametrize("openpilot_long, pcm_cruise_speed, icbm", MODES)
  def test_inactive_model_never_wins(self, openpilot_long, pcm_cruise_speed, icbm):
    p = planner(openpilot_long, pcm_cruise_speed)
    p.scc.vision.output_v_target = VISION
    p.scc.model.output_v_target = V_CRUISE_UNSET
    assert p.scc_vision_v_target == VISION
