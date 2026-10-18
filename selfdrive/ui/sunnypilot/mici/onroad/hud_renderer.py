"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from cereal import custom
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import KM_TO_MILE, HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.text_measure import measure_text_cached

SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
_SLA_ACTIVE_STATES = (SpeedLimitAssistState.active, SpeedLimitAssistState.adapting)

# Sized to read at a glance without crowding the turn intent arrows drawn around that spot.
_SET_SPEED_FONT_SIZE = 56


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

  def _draw_wheel_indicator(self, wheel_txt: rl.Texture, src_rect: rl.Rectangle, dest_rect: rl.Rectangle,
                            origin: tuple[float, float], rotation: float, alpha: int) -> None:
    """The set speed where the wheel would have been green or blue. Every other state is the wheel.

    In ICBM that number has no readout: the car's cluster carries the set speed ICBM rewrites
    several times a second to steer the stock SCC, and the MAX box shows v_cruise for only the
    2.5 s that follow a change to it. self.set_speed is that same v_cruise, reused as is.
    """
    if self._show_wheel_critical or not ui_state.sm['selfdriveState'].enabled:
      super()._draw_wheel_indicator(wheel_txt, src_rect, dest_rect, origin, rotation, alpha)
      return

    set_speed = self.set_speed if ui_state.is_metric else self.set_speed * KM_TO_MILE
    text = str(round(set_speed))
    size = measure_text_cached(self._font_bold, text, _SET_SPEED_FONT_SIZE)
    # dest_rect carries the wheel's centre, since the texture is drawn about its own origin.
    position = rl.Vector2(dest_rect.x - size.x / 2, dest_rect.y - size.y / 2)
    rl.draw_text_ex(self._font_bold, text, position, _SET_SPEED_FONT_SIZE, 0, self._wheel_color(alpha))

  def _wheel_color(self, alpha: int) -> rl.Color:
    if self._show_wheel_critical:
      return rl.Color(255, 255, 255, alpha)  # blanc en mode critique - preserve l'alerte
    if ui_state.sm['selfdriveState'].enabled:
      sla_state = ui_state.sm['longitudinalPlanSP'].speedLimit.assist.state
      if sla_state in _SLA_ACTIVE_STATES:
        return rl.Color(0, 150, 255, alpha)  # bleu - SLA actif
      return rl.Color(0, 220, 0, alpha)      # vert - cruise actif, SLA off
    return rl.Color(255, 255, 255, alpha)    # blanc - cruise desactive
