"""
Etat de la sentinelle, garde dans des fichiers plutot que dans les parametres.

Les cles de parametres vivent dans common/params_keys.h, compile dans une extension
Cython. L'appareil tourne sur une version precompilee, sans SConstruct : une cle ajoutee
y est inconnue, et le premier get_bool la concernant fait tomber le manager
(UnknownKeyName). Des fichiers sous /data font le meme travail, sans rien recompiler,
comme deja fait pour la demande de lecture du diagnostic.

Ce module ne depend que de la bibliotheque standard : il est importe par le manager,
par la surveillance d'alimentation et par l'interface, qui n'ont pas a charger numpy.
"""
from __future__ import annotations

import json
import os

ARMED_PATH = "/data/sentry_armed"
MIN_VOLTAGE_PATH = "/data/sentry_min_voltage"
STATUS_PATH = "/data/sentry_status.json"

# Seuil d'arret par defaut, en millivolts : la valeur openpilot VBATT_PAUSE_CHARGING.
# Hors contact la batterie ne se recharge pas, c'est la seule protection reelle.
DEFAULT_MIN_VOLTAGE_MV = 11800


def is_armed() -> bool:
  return os.path.exists(ARMED_PATH)


def arm() -> None:
  try:
    with open(ARMED_PATH, "w") as f:
      f.write("1")
  except OSError:
    pass


def disarm() -> None:
  try:
    os.unlink(ARMED_PATH)
  except OSError:
    pass


def min_voltage_mv() -> int:
  """Surchargeable sans toucher au code : echo 12200 > /data/sentry_min_voltage"""
  try:
    with open(MIN_VOLTAGE_PATH) as f:
      return int(f.read().strip())
  except (OSError, ValueError):
    return DEFAULT_MIN_VOLTAGE_MV


def read_status() -> dict:
  """Dernier etat publie par sentineld, lu par le panneau. Rien de lourd."""
  try:
    with open(STATUS_PATH) as f:
      return json.load(f)
  except (OSError, ValueError):
    return {}


def write_status(status: dict) -> None:
  try:
    tmp = STATUS_PATH + ".tmp"
    with open(tmp, "w") as f:
      json.dump(status, f)
    os.replace(tmp, STATUS_PATH)
  except OSError:
    pass
