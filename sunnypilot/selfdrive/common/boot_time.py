"""
Reperes pour dater ce qui se produit avant que l'horloge du comma soit a l'heure.

Le comma n'a pas d'horloge sauvegardee : sa puce RTC repart de zero a chaque mise sous
tension, et systemd avance alors l'horloge jusqu'a sa propre date de compilation, celle
de l'image AGNOS. L'heure n'est recalee qu'ensuite, par le wifi ou par le GPS.

Tout ce qui est horodate entre les deux porte donc une date fausse. Le temps monotone,
lui, est juste : associe a l'identifiant du demarrage, il permet de retrouver l'instant
reel une fois l'horloge recalee.
"""
from __future__ import annotations

import time

TIME_FORMAT = "%Y-%m-%d %H:%M:%S"
# au-dela de cet ecart, l'horodatage vient d'une horloge pas encore recalee
TIME_SLACK = 60.


def boot_id() -> str:
  try:
    with open("/proc/sys/kernel/random/boot_id") as f:
      return f.read().strip()
  except OSError:
    return ""


def clock_is_set() -> bool:
  """openpilot sait dire si l'horloge a ete recalee : au boot elle vaut la date de
  compilation de systemd, celle de l'image AGNOS (common/time_helpers.py)."""
  try:
    from openpilot.common.time_helpers import system_time_valid
    return system_time_valid()
  except Exception:
    return False


def real_time(mono: float) -> float:
  """Heure reelle d'un evenement du demarrage courant, d'apres son temps monotone."""
  return time.time() - (time.monotonic() - mono)
