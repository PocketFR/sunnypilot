"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

from collections.abc import Callable

from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigToggle
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.diagnostics import ICON_SIZE, InfoBlock
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.sentinel import state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller


class SentinelLayoutMici(NavScroller):
  """Surveillance de la voiture garee. L'armement redemarre le comma : camerad et
  sentineld ne tournent hors route que si le parametre est pose au demarrage."""

  def __init__(self, back_callback: Callable):
    super().__init__()
    self.set_back_callback(back_callback)

    self._status: dict = {}
    self._icon = gui_app.texture("icons_mici/exclamation_point.png", ICON_SIZE, ICON_SIZE)

    self._state = InfoBlock(tr("sentry"), tr("last event"), scroll_b=False)
    self._power = InfoBlock(tr("battery"), tr("recordings"), scroll_b=False)

    self._arm_btn = BigButton(tr("arm sentry"), "")
    self._arm_btn.set_click_callback(self._arm_prompt)
    self._arm_btn.set_enabled(self._can_arm)

    # Pas de bouton de desarmement : demarrer le moteur est la seule sortie, et c'est
    # ce qui tient lieu d'authentification. Voir sentineld, arret sur contact mis.
    self._auto_btn = BigToggle(tr("arm at engine off"), "", initial_state=state.auto_enabled(),
                               toggle_callback=state.set_auto)

    self._scroller.add_widgets([self._state, self._power, self._arm_btn, self._auto_btn])

  def show_event(self):
    super().show_event()
    self._status = state.read_status()
    self._auto_btn.set_checked(state.auto_enabled())
    self._refresh()

  def _refresh(self) -> None:
    if self._armed():
      state = tr("recording") if self._status.get("recording") else tr("armed")
    else:
      state = tr("off")
    self._state.value_a.set_text(state)
    self._state.value_b.set_text(self._last_event())

    voltage = self._status.get("voltage_mv") or 0
    minimum = self._status.get("min_voltage_mv") or 0
    self._power.value_a.set_text("%.1f V (min %.1f)" % (voltage / 1000., minimum / 1000.) if voltage else "-")

    events, used = self._status.get("events") or 0, self._status.get("bytes") or 0
    self._power.value_b.set_text("%d · %.1f Go" % (events, used / 1e9) if events else tr("none"))

  def _last_event(self) -> str:
    stamp = self._status.get("last_event")
    if not stamp:
      return tr("never")
    return time.strftime("%d/%m/%Y %H:%M", time.localtime(stamp))

  # ------------------------------------------------------------------- actions

  def _armed(self) -> bool:
    return state.is_armed()

  def _can_arm(self) -> bool:
    return not self._armed() and not ui_state.started and not ui_state.engaged

  def _arm(self) -> None:
    # Pas de redemarrage : le manager reevalue les portes a chaque tour de boucle, donc
    # camerad et sentineld demarrent en une seconde, comme le fait l'armement automatique.
    state.arm()

  def _arm_prompt(self) -> None:
    gui_app.push_widget(BigConfirmationDialog(tr("slide to arm sentry"), self._icon,
                                              confirm_callback=self._arm))
