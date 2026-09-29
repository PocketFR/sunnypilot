"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V

# The vision controller derives its target from the predicted lateral acceleration, which gives
# v_target = v_ego * sqrt(A_LAT_MAX / a_lat_pred) - strictly proportional to the current speed,
# so it cannot say anything until the wheel is already turning. Measured approaching a
# roundabout: it only produced a target below the current speed 1.7 s before the apex, and just
# before that it was asking for 84.5 km/h while the car was doing 73.5.
#
# The model already plans the slowdown far earlier - it had dropped its six second minimum to
# 50 km/h some 13.5 s ahead of that same roundabout, and on lead-car decelerations it crosses
# -15 % a mean 9.8 s before the low point. That trajectory is decoded and published every frame
# and nothing reads it as a speed target. This controller does.
HORIZON = 6.0  # s, how far down the planned trajectory we look

# Gate on a sustained planned deceleration rather than on the speed alone, so that ordinary
# trajectory noise does not switch the candidate on and off. Hysteresis band, since each
# oscillation of the published target costs real button presses on the cluster.
DECEL_ON = -0.35   # m/s^2
DECEL_OFF = -0.15  # m/s^2

# A planned deceleration is not enough on its own: replaying a real approach, the acceleration
# gate alone flickered on and off and published targets *above* the current speed (76 km/h while
# doing 68.7), which would still have won the min() against the driver's 80 and cut the set
# speed for nothing. The planned minimum must also sit meaningfully below where we already are.
# Tuned by replay on route 00000336 (1272 s engaged). The candidate costs button presses on
# the cluster every time it engages and releases, so the threshold trades anticipation against
# that traffic: 3 km/h -> 1638 presses, 8 -> 1355, 12 -> 1091, against 683 for the current
# behaviour. 8 km/h was kept because it still catches the roundabout approach that motivated
# this work, only 0.2 s later than 3 km/h would. Raise it if the cluster proves too busy.
MARGIN_ON = 8 * CV.KPH_TO_MS   # m/s
MARGIN_OFF = 1 * CV.KPH_TO_MS  # m/s

# The model's plan wavers around the gate from frame to frame. Without a hold, replaying a real
# approach showed the candidate toggling every two or three frames, and since each toggle moves
# the published target by more than 10 km/h, ICBM would have chased it up and down for nothing.
# Once engaged, the release condition must therefore hold continuously for this long.
RELEASE_HOLD = 2.0  # s

# Smooths the 10-20 Hz jitter measured on the vision controller (+/- 5 km/h within a second)
# without materially delaying an anticipation that arrives seconds ahead.
FILTER_RC = 1.0  # s


class SmartCruiseControlModel:
  """Turns the model's own planned speed trajectory into a vTarget candidate."""

  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.enabled = self.params.get_bool("SmartCruiseControlVision")

    self.long_enabled = False
    self.long_override = False
    self.v_ego = 0.0
    self.is_enabled = False
    self.is_active = False

    self.v_planned = 0.0
    self.a_planned = 0.0
    self.output_v_target = V_CRUISE_UNSET
    self.output_a_target = 0.0

    self.release_frames = 0
    self.horizon_idx = int(np.searchsorted(ModelConstants.T_IDXS, HORIZON))
    self.v_filter = FirstOrderFilter(0.0, FILTER_RC, DT_MDL)

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      # Shares the Smart Cruise Control vision switch: both read the model, and no new param
      # key can be declared on a prebuilt device (params_keys.h is compiled into params_pyx).
      self.enabled = self.params.get_bool("SmartCruiseControlVision")

  def _update_calculations(self, sm: messaging.SubMaster) -> bool:
    """Returns True when the model trajectory is usable this frame."""
    md = sm['modelV2']
    if len(md.velocity.x) != ModelConstants.IDX_N or len(md.acceleration.x) != ModelConstants.IDX_N:
      return False

    # Same length guard as parse_model in selfdrive/controls/lib/longitudinal_planner.py.
    self.v_planned = float(np.min(md.velocity.x[:self.horizon_idx]))
    self.a_planned = float(np.min(md.acceleration.x[:self.horizon_idx]))
    return True

  def _update_state(self, usable: bool) -> None:
    self.is_enabled = self.enabled and self.long_enabled and not self.long_override

    # Losing the engagement, the pedal or the trajectory drops out at once, no hold.
    if not (self.is_enabled and usable) or self.v_ego <= MIN_V:
      self.is_active = False
      self.release_frames = 0
      return

    # Hysteresis on both terms: arm on a clear planned deceleration that also lands clearly
    # below the current speed, release as soon as either condition has plainly gone.
    if self.a_planned < DECEL_ON and self.v_planned < self.v_ego - MARGIN_ON:
      self.is_active = True
      self.release_frames = 0
    elif self.a_planned > DECEL_OFF or self.v_planned > self.v_ego - MARGIN_OFF:
      self.release_frames += 1
      if self.release_frames >= int(RELEASE_HOLD / DT_MDL):
        self.is_active = False
    else:
      self.release_frames = 0

  def get_v_target_from_control(self) -> float:
    if self.is_active:
      return max(self.v_filter.x, MIN_V)

    return V_CRUISE_UNSET

  def get_a_target_from_control(self) -> float:
    return self.output_a_target

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool,
             v_ego: float, a_ego: float, v_cruise: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego

    self._update_params()
    usable = self._update_calculations(sm)
    was_active = self.is_active
    self._update_state(usable)

    if self.is_active and usable:
      if not was_active:
        # Start from the current speed rather than from a stale filter state, otherwise the
        # first published target would jump and cost a burst of presses for nothing. The gate
        # guarantees v_planned is already below v_ego here, so this only ever decays downwards.
        self.v_filter.x = v_ego
      self.v_filter.update(self.v_planned)
    else:
      self.v_filter.x = v_ego

    self.output_a_target = a_ego
    self.output_v_target = self.get_v_target_from_control()

    self.frame += 1
