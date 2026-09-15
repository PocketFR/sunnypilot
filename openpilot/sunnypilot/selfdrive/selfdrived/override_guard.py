#!/usr/bin/env python3
"""
Garde-fou "override longitudinal".

Quand la pedale d'accelerateur est enfoncee, openpilot coupe son controle
longitudinal (controlsd.py: CC.longActive = CC.enabled and not any(
e.overrideLongitudinal for e in onroadEvents)). Cet etat est totalement
silencieux: l'evenement gasPressedOverride est declare avec AlertSize.none et
AudibleAlert.none, donc rien n'indique au conducteur que le freinage automatique
est indisponible. Si une cible ralentit devant, plus rien n'intervient jusqu'au
FCW du planificateur, qui se declenche sur crash_cnt > 2 et arrive donc tres
tard.

Ce module leve une alerte graduee des que l'override dure ET qu'une cible se
rapproche. Il ne prend aucune autorite sur les actionneurs: il ne fait
qu'alerter.

Seuils valides par rejeu sur les rlogs (route 000002de, Ioniq PHEV):
  - declenchement WARNING a t-8.9 s de l'impact, contre t-1.2 s pour le FCW
  - 0 WARNING sur 64 min d'autres trajets (routes 2da, 2db, 2dc, 2df)
"""
from cereal import car, log
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import Alert, Priority

AlertStatus = log.SelfdriveState.AlertStatus
AlertSize = log.SelfdriveState.AlertSize
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert


def _alert(alert_type: str, text_1: str, text_2: str, status, size, priority,
           visual, audible, duration: float) -> Alert:
  a = Alert(text_1, text_2, status, size, priority, visual, audible, duration)
  # AlertManager indexe par alert_type (str) : pas besoin d'un EventName capnp
  a.alert_type = alert_type
  return a


def advisory_alert() -> Alert:
  return _alert("overrideGuardAdvisory",
                "Accelerator Held - No Auto Braking", "",
                AlertStatus.userPrompt, AlertSize.small, Priority.MID,
                VisualAlert.none, AudibleAlert.none, 1.)


def warning_alert() -> Alert:
  return _alert("overrideGuardWarning",
                "RELEASE ACCELERATOR", "Lead Vehicle - No Auto Braking",
                AlertStatus.critical, AlertSize.full, Priority.HIGHEST,
                VisualAlert.fcw, AudibleAlert.warningImmediate, 2.)


class OverrideGuard:
  HOLD_ADVISORY = 8.0     # s d'appui continu avant l'alerte visuelle
  HOLD_WARNING = 5.0      # s d'appui continu avant l'alerte sonore
  HEADWAY_ADVISORY = 1.2  # s
  HEADWAY_WARNING = 0.9   # s
  TTC_WARNING = 4.0       # s
  # 3.0 m/s (~11 km/h) : un choc arriere a 10-15 km/h de vitesse de
  # rapprochement fait deja des degats, donc on reste actif bas. Mesure sur
  # 81 min de logs : passer de 5.0 a 3.0 ne change aucun declenchement, la
  # condition d'appui continu >5 s n'etant pas remplie dans les bouchons.
  V_MIN = 3.0             # m/s, en dessous on considere une manoeuvre
  PROB_MIN = 0.5          # confiance minimale sur la cible

  # l'advisory est informatif: on l'affiche brievement puis on le rearme, pour
  # ne pas le laisser en permanence pendant un appui long (mesure: 48.7 s
  # d'affichage continu sur un trajet reel avec un affichage latche)
  ADVISORY_SHOW = 3.0     # s d'affichage
  ADVISORY_REARM = 60.0   # s avant un nouvel affichage

  def __init__(self):
    self.gas_held = 0.
    self.adv_show_left = 0.
    self.adv_rearm_left = 0.
    self.alert: Alert | None = None

  def update(self, CS, sm, enabled: bool, CP) -> None:
    self.alert = None

    # sans controle longitudinal openpilot, l'ACC/AEB d'origine reste actif:
    # ce garde-fou ne s'applique pas
    if not CP.openpilotLongitudinalControl:
      return

    self.adv_rearm_left = max(0., self.adv_rearm_left - DT_CTRL)

    if not (enabled and CS.gasPressed):
      self.gas_held = 0.
      self.adv_show_left = 0.
      return

    self.gas_held += DT_CTRL

    lead = sm['radarState'].leadOne
    if not lead.status or lead.modelProb < self.PROB_MIN or CS.vEgo < self.V_MIN:
      return

    headway = lead.dRel / CS.vEgo
    closing = -lead.vRel
    ttc = lead.dRel / closing if closing > 0.5 else float('inf')

    if self.gas_held > self.HOLD_WARNING and (headway < self.HEADWAY_WARNING or ttc < self.TTC_WARNING):
      # le warning reste affiche tant que la situation dure
      self.alert = warning_alert()
      return

    if self.gas_held > self.HOLD_ADVISORY and headway < self.HEADWAY_ADVISORY:
      if self.adv_show_left <= 0. and self.adv_rearm_left <= 0.:
        self.adv_show_left = self.ADVISORY_SHOW
        self.adv_rearm_left = self.ADVISORY_REARM

    if self.adv_show_left > 0.:
      self.adv_show_left -= DT_CTRL
      self.alert = advisory_alert()

  def alerts(self) -> list[Alert]:
    return [self.alert] if self.alert is not None else []
