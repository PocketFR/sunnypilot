"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import car, custom
from opendbc.car import structs, apply_hysteresis
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

ALLOWED_SPEED_THRESHOLD = 1.8  # m/s, ~4 MPH
HYST_GAP = 0.0  # currently disabled; TODO-SP: might need to be brand-specific
INACTIVE_TIMER = 0.4

# While the driver holds the accelerator, ICBM is held inactive and the set speed freezes as
# the car speeds up - measured up to 23.8 km/h of gap over a single 10 s pull, and the stock
# SCC brakes on lift-off to get back down to it. On this platform a SET- press taken while
# overriding does not decrement: it re-anchors the set speed onto the indicated speed. Logged
# on 2026-09-24: set speed 32, vEgoCluster 53, one press -> 53. So one press closes the gap
# that would otherwise cost a five second climb at ~5 km/h/s.
#
# Deliberately fire-once-and-do-not-insist: if a car does not re-anchor, the single press just
# decrements by one and nothing worse happens.
GAS_RESYNC_THRESHOLD = 5   # km/h or mph, gap above which one resync press is worth it
GAS_RESYNC_REARM = 2       # gap below which we consider the resync done and re-arm
GAS_RESYNC_PRESS = 0.20    # s; icbm.py emits its burst between t+0.10 and t+0.15 after the
                           # previous press, so the request must be held across that window


SEND_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0

    self.is_ready = False
    self.is_ready_prev = False
    self.v_target_ms_last = 0.0
    self.is_metric = False

    self.cruise_button_timers = CRUISE_BUTTON_TIMER

    self.v_ego_cluster = 0
    self.gas_resync_armed = True
    self.gas_resync_frames = 0

  @property
  def v_cruise_equal(self) -> bool:
    return self.v_target == self.v_cruise_cluster

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    ms_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    self.v_target_ms_last = apply_hysteresis(LP_SP.vTarget, self.v_target_ms_last, HYST_GAP * ms_conv)

    self.v_target = round(self.v_target_ms_last * speed_conv)
    self.v_cruise_min = get_minimum_set_speed(self.is_metric)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)
    # Cluster speed, not vEgo: the set speed re-anchors onto what the dial shows (logged 53,
    # with vEgo at 48.7 at the same instant).
    self.v_ego_cluster = round(CS.vEgoCluster * speed_conv)

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    # HOLDING, ACCELERATING, DECELERATING, PRE_ACTIVE
    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive

      else:
        # PRE_ACTIVE
        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_cruise_equal:
              self.state = State.holding

            elif self.v_target > self.v_cruise_cluster:
              self.state = State.increasing

            elif self.v_target < self.v_cruise_cluster and self.v_cruise_cluster > self.v_cruise_min:
              self.state = State.decreasing

        # HOLDING
        elif self.state == State.holding:
          if not self.v_cruise_equal:
            self.state = State.preActive

        # ACCELERATING
        elif self.state == State.increasing:
          if self.v_target <= self.v_cruise_cluster:
            self.state = State.holding

        # DECELERATING
        elif self.state == State.decreasing:
          if self.v_target >= self.v_cruise_cluster or self.v_cruise_cluster <= self.v_cruise_min:
            self.state = State.holding

    # INACTIVE
    elif self.state == State.inactive:
      if self.is_ready and not self.is_ready_prev:
        self.pre_active_timer = int(INACTIVE_TIMER / DT_CTRL)
        self.state = State.preActive

    send_button = SEND_BUTTONS.get(self.state, SendButtonState.none)

    return send_button

  def update_gas_override_resync(self, CC: car.CarControl) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    """One SET- press to re-anchor the set speed while the driver accelerates. See the constants above."""
    if self.gas_resync_frames > 0:
      self.gas_resync_frames -= 1
      return SendButtonState.decrease

    overriding = CC.enabled and CC.cruiseControl.override
    gap = self.v_ego_cluster - self.v_cruise_cluster

    if not overriding or gap <= GAS_RESYNC_REARM:
      self.gas_resync_armed = True
      return SendButtonState.none

    if self.gas_resync_armed and gap >= GAS_RESYNC_THRESHOLD:
      self.gas_resync_armed = False
      self.gas_resync_frames = int(GAS_RESYNC_PRESS / DT_CTRL) - 1
      return SendButtonState.decrease

    return SendButtonState.none

  def update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = CC.enabled and not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume
    button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    self.is_ready = ready and not button_pressed

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, is_metric: bool) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      return

    self.is_metric = is_metric

    self.update_calculations(CS, LP_SP)
    self.update_readiness(CS, CC)

    self.cruise_button = self.update_state_machine()

    # The state machine yields nothing while overriding, so the resync takes precedence there.
    resync_button = self.update_gas_override_resync(CC)
    if resync_button != SendButtonState.none:
      self.cruise_button = resync_button

    self.is_ready_prev = self.is_ready
