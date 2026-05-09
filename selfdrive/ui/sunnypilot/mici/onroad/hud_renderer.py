"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from cereal import custom
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.ui_state import ui_state

SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
_SLA_ACTIVE_STATES = (SpeedLimitAssistState.active, SpeedLimitAssistState.adapting)


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)

  def _has_blind_spot_detected(self) -> bool:
    return self.blind_spot_indicators.detected

  def _wheel_color(self, alpha: int) -> rl.Color:
    if self._show_wheel_critical:
      return rl.Color(255, 255, 255, alpha)  # blanc en mode critique - preserve l'alerte
    if ui_state.sm['selfdriveState'].enabled:
      sla_state = ui_state.sm['longitudinalPlanSP'].speedLimit.assist.state
      if sla_state in _SLA_ACTIVE_STATES:
        return rl.Color(0, 150, 255, alpha)  # bleu - SLA actif
      return rl.Color(0, 220, 0, alpha)      # vert - cruise actif, SLA off
    return rl.Color(255, 255, 255, alpha)    # blanc - cruise desactive
