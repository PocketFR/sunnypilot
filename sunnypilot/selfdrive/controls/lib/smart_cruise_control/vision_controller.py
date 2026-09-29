"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import cereal.messaging as messaging
from cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState

ACTIVE_STATES = (VisionState.entering, VisionState.turning, VisionState.leaving)
ENABLED_STATES = (VisionState.enabled, VisionState.overriding, *ACTIVE_STATES)

_ENTERING_PRED_LAT_ACC_TH = 1.3  # Predicted Lat Acc threshold to trigger entering turn state.
_ABORT_ENTERING_PRED_LAT_ACC_TH = 1.1  # Predicted Lat Acc threshold to abort entering state if speed drops.

_TURNING_LAT_ACC_TH = 1.6  # Lat Acc threshold to trigger turning state.

_LEAVING_LAT_ACC_TH = 1.3  # Lat Acc threshold to trigger leaving turn state.
_FINISH_LAT_ACC_TH = 1.1  # Lat Acc threshold to trigger the end of the turn cycle.

# Lateral acceleration budget, by the radius of the curve ahead. A single flat 2.0 m/s^2 is
# about right for tight corners but far too cautious on open bends: measured over two crossings
# of the Col de la Republique (route 0000034c and 0000034f, 2026-09-28), the driver held a
# median 4.17 m/s^2 through 40-60 m radius curves and 2.94 through 60-100 m, driving 15 and
# 10 km/h faster than this controller was asking for. In tight corners elsewhere - roundabouts
# and town junctions, 8 to 40 m - the same driver was only 3 to 4 km/h above it.
#
# The envelope itself is not what differs: at 40-60 m the 90th percentile is 4.54 on the pass
# against 5.05 elsewhere. What differs is how often it is used. So indexing the budget on the
# radius alone is legitimate, and no road-class signal is needed - which is just as well, since
# the speed limit is only valid 7 % of the time on these roads.
#
# Held flat below 25 m so that roundabouts keep exactly the target they have today.
_A_LAT_RADIUS_BP = [25., 60., 120.]  # m, radius of the curve ahead
_A_LAT_RADIUS_V = [2.0, 3.0, 3.0]    # m/s^2, budget at those radii

_NO_OVERSHOOT_TIME_HORIZON = 4.  # s. Time to use for velocity desired based on a_target when not overshooting.

# Lookup table for the minimum smooth deceleration during the ENTERING state
# depending on the actual maximum absolute lateral acceleration predicted on the turn ahead.
_ENTERING_SMOOTH_DECEL_V = [-0.2, -1.]  # min decel value allowed on ENTERING state
_ENTERING_SMOOTH_DECEL_BP = [1.3, 3.]  # absolute value of lat acc ahead

# Lookup table for the acceleration for the TURNING state
# depending on the current lateral acceleration of the vehicle.
_TURNING_ACC_V = [0.5, 0., -0.4]  # acc value
_TURNING_ACC_BP = [1.5, 2.3, 3.]  # absolute value of current lat acc

_LEAVING_ACC = 0.5  # Conformable acceleration to regain speed while leaving a turn.


class SmartCruiseControlVision:
  v_target: float = 0
  a_target: float = 0.
  v_ego: float = 0.
  a_ego: float = 0.
  output_v_target: float = V_CRUISE_UNSET
  output_a_target: float = 0.

  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    self.enabled = self.params.get_bool("SmartCruiseControlVision")
    self.v_cruise_setpoint = 0.

    self.state = VisionState.disabled
    self.current_lat_acc = 0.
    self.max_pred_lat_acc = 0.

  def get_a_target_from_control(self) -> float:
    return self.a_target

  def get_v_target_from_control(self) -> float:
    if self.is_active:
      return max(self.v_target, MIN_V) + self.a_target * _NO_OVERSHOOT_TIME_HORIZON

    return V_CRUISE_UNSET

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartCruiseControlVision")

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    if not self.long_enabled:
      return
    else:
      rate_plan = np.array(np.abs(sm['modelV2'].orientationRate.z))
      vel_plan = np.array(sm['modelV2'].velocity.x)

      self.current_lat_acc = self.v_ego ** 2 * abs(sm['controlsState'].curvature)

      # get the maximum lat accel from the model
      predicted_lat_accels = rate_plan * vel_plan
      self.max_pred_lat_acc = np.percentile(predicted_lat_accels, 97)

      # get the maximum curve based on the current velocity
      v_ego = max(self.v_ego, 0.1)  # ensure a value greater than 0 for calculations
      max_curve = self.max_pred_lat_acc / (v_ego**2)

      # Get the target velocity for the maximum curve, on a budget that opens up as the curve
      # opens up. np.interp clamps, so anything tighter than 25 m keeps the original 2.0 and
      # anything more open than 120 m sits at 3.0 - by which radius the curve has stopped being
      # the binding constraint anyway, the speed limit or the driver's set speed winning first.
      radius = 1.0 / max(max_curve, 1e-6)
      a_lat_max = float(np.interp(radius, _A_LAT_RADIUS_BP, _A_LAT_RADIUS_V))
      self.v_target = (a_lat_max / max_curve) ** 0.5

  def _update_state_machine(self) -> tuple[bool, bool]:
    # ENABLED, ENTERING, TURNING, LEAVING, OVERRIDING
    if self.state != VisionState.disabled:
      # longitudinal and feature disable always have priority in a non-disabled state
      if not self.long_enabled or not self.enabled:
        self.state = VisionState.disabled
      elif self.long_override:
        self.state = VisionState.overriding

      else:
        # ENABLED
        if self.state == VisionState.enabled:
          # Do not enter a turn control cycle if the speed is low.
          if self.v_ego <= MIN_V:
            pass
          # If significant lateral acceleration is predicted ahead, then move to Entering turn state.
          elif self.max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH:
            self.state = VisionState.entering

        # OVERRIDING
        elif self.state == VisionState.overriding:
          if not self.long_override:
            self.state = VisionState.enabled

        # ENTERING
        elif self.state == VisionState.entering:
          # Transition to Turning if current lateral acceleration is over the threshold.
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          # Abort if the predicted lateral acceleration drops
          elif self.max_pred_lat_acc < _ABORT_ENTERING_PRED_LAT_ACC_TH:
            self.state = VisionState.enabled

        # TURNING
        elif self.state == VisionState.turning:
          # Transition to Leaving if current lateral acceleration drops below a threshold.
          if self.current_lat_acc <= _LEAVING_LAT_ACC_TH:
            self.state = VisionState.leaving

        # LEAVING
        elif self.state == VisionState.leaving:
          # Transition back to Turning if current lateral acceleration goes back over the threshold.
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          # Finish if current lateral acceleration goes below a threshold.
          elif self.current_lat_acc < _FINISH_LAT_ACC_TH:
            self.state = VisionState.enabled

    # DISABLED
    elif self.state == VisionState.disabled:
      if self.long_enabled and self.enabled:
        if self.long_override:
          self.state = VisionState.overriding
        else:
          self.state = VisionState.enabled

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def _update_solution(self) -> float:
    # DISABLED, ENABLED, OVERRIDING
    if self.state not in ACTIVE_STATES:
      # when not overshooting, calculate v_turn as the speed at the prediction horizon when following
      # the smooth deceleration.
      a_target = self.a_ego
    # ENTERING
    elif self.state == VisionState.entering:
      # when not overshooting, target a smooth deceleration in preparation for a sharp turn to come.
      a_target = np.interp(self.max_pred_lat_acc, _ENTERING_SMOOTH_DECEL_BP, _ENTERING_SMOOTH_DECEL_V)
    # TURNING
    elif self.state == VisionState.turning:
      # When turning, we provide a target acceleration that is comfortable for the lateral acceleration felt.
      a_target = np.interp(self.current_lat_acc, _TURNING_ACC_BP, _TURNING_ACC_V)
    # LEAVING
    elif self.state == VisionState.leaving:
      # When leaving, we provide a comfortable acceleration to regain speed.
      a_target = _LEAVING_ACC
    else:
      raise NotImplementedError(f"SCC-V state not supported: {self.state}")

    return a_target

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
             v_cruise_setpoint: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise_setpoint = v_cruise_setpoint

    self._update_params()
    self._update_calculations(sm)

    self.is_enabled, self.is_active = self._update_state_machine()
    self.a_target = self._update_solution()

    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    self.frame += 1
