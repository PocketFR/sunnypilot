"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import subprocess
import time

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.diagnostics import obd_dtc
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult, Widget
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.list_view import button_item, text_item
from openpilot.system.ui.widgets.scroller_tici import Scroller

if gui_app.sunnypilot_ui():
  from openpilot.system.ui.sunnypilot.widgets.list_view import button_item_sp as button_item

DESCRIPTIONS = {
  'read': tr_noop("Read the engine fault codes. The panda can only have one master, so openpilot restarts and reads "
                  "them at startup, before it takes control of the car."),
  'clear': tr_noop("Erase the stored fault codes of the engine, the cruise control radar, the ABS and the power "
                   "steering. The last three log a lost communication every time openpilot silences the radar to "
                   "control the brakes. This also resets the emissions readiness monitors, which can fail the OBD "
                   "part of a roadworthiness test until the car has completed its drive cycles."),
}


class DiagnosticsLayout(Widget):
  def __init__(self):
    super().__init__()
    self._status = obd_dtc.read_status()
    self._scroller = Scroller(self._initialize_items(), line_separator=True, spacing=0)

  def _initialize_items(self):
    return [
      text_item(lambda: tr("Check Engine Light"), self._mil),
      text_item(lambda: tr("Stored Codes"), lambda: self._codes("stored")),
      text_item(lambda: tr("Pending Codes"), lambda: self._codes("pending")),
      text_item(lambda: tr("Failed Since Last Clear"), lambda: self._codes("failed_since_clear")),
      text_item(lambda: tr("Other ECUs"), self._other_ecus),
      text_item(lambda: tr("Last Read"), self._last_read),
      button_item(lambda: tr("Read Codes"), lambda: tr("READ"), lambda: tr(DESCRIPTIONS['read']),
                  callback=self._read_prompt, enabled=self._can_act),
      button_item(lambda: tr("Clear Codes"), lambda: tr("CLEAR"), lambda: tr(DESCRIPTIONS['clear']),
                  callback=self._clear_prompt, enabled=self._can_clear),
    ]

  def show_event(self):
    super().show_event()
    self._status = obd_dtc.read_status()
    self._scroller.show_event()

  def _render(self, rect):
    self._scroller.render(rect)

  # ------------------------------------------------------------------ affichage

  def _mil(self) -> str:
    if not self._status.get("ok"):
      return tr("N/A")
    return tr("ON") if self._status.get("mil") else tr("off")

  def _codes(self, key: str) -> str:
    if not self._status.get("ok"):
      return tr("N/A")
    return ", ".join(self._status.get(key) or []) or tr("none")

  def _other_ecus(self) -> str:
    others = self._status.get("other_ecus") or {}
    return " / ".join("%s %s" % (n, ",".join(c)) for n, c in sorted(others.items())) or tr("none")

  def _last_read(self) -> str:
    if not self._status:
      return tr("never")
    stamp = time.strftime("%d/%m/%Y %H:%M", time.localtime(self._status.get("time", 0)))
    if not self._status.get("ok"):
      return "%s (%s)" % (stamp, self._status.get("error") or tr("failed"))
    return stamp

  # ------------------------------------------------------------------- actions

  def _stopped(self) -> bool:
    try:
      return ui_state.sm["carState"].vEgo < 0.5
    except Exception:
      return True

  def _can_act(self) -> bool:
    return self._stopped() and not ui_state.engaged

  def _clearable(self) -> list[str]:
    """Ce que l'effacement vise : le moteur, plus les calculateurs chassis (obd_dtc.CLEAR_ECUS)."""
    others = self._status.get("other_ecus") or {}
    engine = sorted(set(self._status.get("stored") or []) | set(self._status.get("confirmed") or []) |
                    set(self._status.get("pending") or []))
    return engine + ["%s %s" % (name, code)
                     for name in obd_dtc.CHASSIS_ECUS.values() for code in others.get(name, [])]

  def _can_clear(self) -> bool:
    return self._can_act() and bool(self._clearable() or self._status.get("mil"))

  def _restart(self, action: str) -> None:
    obd_dtc.request(action)
    subprocess.Popen(["sudo", "systemctl", "restart", "comma"])

  def _read_prompt(self) -> None:
    def on_result(result: DialogResult):
      if result == DialogResult.CONFIRM:
        self._restart("read")

    gui_app.push_widget(ConfirmDialog(tr("Restart openpilot and read the fault codes?"), tr("Restart"),
                                      callback=on_result))

  def _clear_prompt(self) -> None:
    codes = self._clearable()
    text = tr("Erase these codes?") + "\n\n" + (", ".join(codes) if codes else tr("none")) + "\n\n" + \
           tr("openpilot will restart. The emissions readiness monitors will be reset.")

    def on_result(result: DialogResult):
      if result == DialogResult.CONFIRM:
        self._restart("clear")

    gui_app.push_widget(ConfirmDialog(text, tr("Erase"), callback=on_result))
