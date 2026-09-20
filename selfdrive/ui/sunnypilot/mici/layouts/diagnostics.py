"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import subprocess
import time

from collections.abc import Callable

import pyray as rl

from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.diagnostics import obd_dtc
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.system.ui.widgets.scroller import NavScroller

ICON_SIZE = 110


class InfoBlock(Widget):
  """Deux lignes titre + valeur, au format des blocs d'info mici."""

  def __init__(self, header_a: str, header_b: str, scroll_b: bool = True):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, 402, 180))

    header_color = rl.Color(255, 255, 255, int(255 * 0.9))
    value_color = rl.Color(255, 255, 255, int(255 * 0.9 * 0.65))
    max_width = int(self._rect.width - 20)

    self._header_a = UnifiedLabel(header_a, 48, max_width=max_width, text_color=header_color,
                                  font_weight=FontWeight.DISPLAY, shimmer=True)
    self.value_a = UnifiedLabel("N/A", 32, max_width=max_width, text_color=value_color,
                                font_weight=FontWeight.ROMAN)
    self._header_b = UnifiedLabel(header_b, 48, max_width=max_width, text_color=header_color,
                                  font_weight=FontWeight.DISPLAY, shimmer=True)
    self.value_b = UnifiedLabel("N/A", 32, max_width=max_width, text_color=value_color,
                                font_weight=FontWeight.ROMAN, scroll=scroll_b)

  def _render(self, _):
    self._header_a.set_position(self._rect.x + 20, self._rect.y - 10)
    self._header_a.render()
    self.value_a.set_position(self._rect.x + 20, self._rect.y + 68 - 25)
    self.value_a.render()
    self._header_b.set_position(self._rect.x + 20, self._rect.y + 114 - 30)
    self._header_b.render()
    self.value_b.set_position(self._rect.x + 20, self._rect.y + 161 - 25)
    self.value_b.render()


class DiagnosticsLayoutMici(NavScroller):
  """Codes defaut. La lecture se fait au demarrage d'openpilot (voir obd_dtc), seul
  moment ou le panda est libre : les boutons posent un drapeau puis relancent."""

  def __init__(self, back_callback: Callable):
    super().__init__()
    self.set_back_callback(back_callback)

    self._status: dict = {}
    self._icon = gui_app.texture("icons_mici/exclamation_point.png", ICON_SIZE, ICON_SIZE)

    self._engine = InfoBlock(tr("check engine"), tr("engine codes"))
    self._others = InfoBlock(tr("other ecus"), tr("last read"), scroll_b=False)

    self._read_btn = BigButton(tr("read codes"), "")
    self._read_btn.set_click_callback(self._read_prompt)
    self._read_btn.set_enabled(self._can_act)

    self._clear_btn = BigButton(tr("clear codes"), "")
    self._clear_btn.set_click_callback(self._clear_prompt)
    self._clear_btn.set_enabled(self._can_clear)

    self._scroller.add_widgets([self._engine, self._others, self._read_btn, self._clear_btn])

  def show_event(self):
    super().show_event()
    self._status = obd_dtc.read_status()
    self._refresh()

  def _refresh(self) -> None:
    self._others.value_b.set_text(self._last_read())

    if not self._status.get("ok"):
      self._engine.value_a.set_text(tr("no reading"))
      self._engine.value_b.set_text(self._status.get("error") or tr("read at every ignition on"))
      self._others.value_a.set_text("-")
      return

    self._engine.value_a.set_text(tr("on") if self._status.get("mil") else tr("off"))
    self._engine.value_b.set_text(", ".join(self._codes()) or tr("none"))

    others = self._status.get("other_ecus") or {}
    self._others.value_a.set_text(
      " / ".join("%s %s" % (name, ",".join(codes)) for name, codes in sorted(others.items())) or tr("none"))

  def _last_read(self) -> str:
    stamp = self._status.get("time")
    if not stamp:
      return tr("never")
    return time.strftime("%d/%m/%Y %H:%M", time.localtime(stamp))

  def _codes(self) -> list[str]:
    return sorted(set(self._status.get("stored") or []) | set(self._status.get("confirmed") or []) |
                  set(self._status.get("pending") or []))

  def _clearable(self) -> list[str]:
    """Ce que l'effacement vise : le moteur, plus les calculateurs chassis (obd_dtc.CLEAR_ECUS)."""
    others = self._status.get("other_ecus") or {}
    return self._codes() + ["%s %s" % (name, code)
                            for name in obd_dtc.CHASSIS_ECUS.values() for code in others.get(name, [])]

  # ------------------------------------------------------------------- actions

  def _stopped(self) -> bool:
    try:
      return ui_state.sm["carState"].vEgo < 0.5
    except Exception:
      return True

  def _can_act(self) -> bool:
    return self._stopped() and not ui_state.engaged

  def _can_clear(self) -> bool:
    return self._can_act() and bool(self._clearable() or self._status.get("mil"))

  def _restart(self, action: str) -> None:
    obd_dtc.request(action)
    subprocess.Popen(["sudo", "systemctl", "restart", "comma"])

  def _read_prompt(self) -> None:
    gui_app.push_widget(BigConfirmationDialog(tr("slide to restart and read codes"), self._icon,
                                              confirm_callback=lambda: self._restart("read")))

  def _clear_prompt(self) -> None:
    codes = ", ".join(self._clearable()) or tr("none")
    gui_app.push_widget(BigConfirmationDialog(tr("slide to erase") + " " + codes, self._icon, red=True,
                                              confirm_callback=lambda: self._restart("clear")))
