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
RUNNING_PATH = "/data/sentry_running"
AUTO_PATH = "/data/sentry_auto"
MIN_VOLTAGE_PATH = "/data/sentry_min_voltage"
MOTION_DELTA_PATH = "/data/sentry_motion_delta"
MOTION_AREA_PATH = "/data/sentry_motion_area"
MOTION_HOLD_PATH = "/data/sentry_motion_hold"
MAX_GB_PATH = "/data/sentry_max_gb"
SHOCK_PATH = "/data/sentry_shock_ms2"
STATUS_PATH = "/data/sentry_status.json"

# Seuil d'arret par defaut, en millivolts : la valeur openpilot VBATT_PAUSE_CHARGING.
# Hors contact la batterie ne se recharge pas, c'est la seule protection reelle.
DEFAULT_MIN_VOLTAGE_MV = 11800

# Mouvement : ecart d'intensite d'une cellule, part des cellules qui doivent bouger, et
# nombre d'images consecutives qui doivent la depasser.
#
# Mesure sur des evenements reels : une personne qui tourne autour de la voiture donne
# 14,5 % de cellules en mouvement. Le vent dans les arbres donnait 0,44 % au pire par
# temps calme, mais une matinee ensoleillee et ventee devant une haie qui remplit le
# cadre monte a 12,4 % en pointe (608 images mesurees le 21/09/2026) : l'amplitude seule
# ne separe plus rien. Ce qui les separe, c'est la duree. Sur ces memes images, aucune
# rafale ne tient trois images d'affilee au-dessus de 3 %, alors qu'une personne y reste
# plusieurs secondes. Un saut d'exposition automatique, qui ne dure qu'une image, tombe
# par la meme regle.
DEFAULT_MOTION_DELTA = 12
DEFAULT_MOTION_AREA = 0.03
DEFAULT_MOTION_HOLD = 3

# La camera cabine ne voit l'exterieur que par un coin de son champ : son fisheye est
# rempli par l'habitacle. Mesure du 21/09/2026, quelqu'un debout a la vitre conducteur :
# 3,4 % du cadre change, 1,6 % apres groupement, la ou la meme personne devant la voiture
# donne 22 % et 11 % sur la camera avant. Au seuil commun elle ne declenche donc jamais.
# A 1,2 % sur deux images elle voit l'arrivee a la vitre, pour deux fausses alertes sur les
# 55 evenements de vent de cette matinee. Les deux autres cameras gardent le seuil commun.
CAMERA_MOTION = {"d": (0.012, 2)}

# Place reservee aux evenements. Mesure du 21/09/2026 : 106 evenements pesent 1,6 Go, soit
# ~15 Mo piece. Le disque fait 94,5 Go, dont 70 Go de trajets ; ce que la sentinelle prend
# en plus, le nettoyeur des trajets le reprend sur les segments les plus anciens, qui sont
# deja sur le NAS.
DEFAULT_MAX_GB = 3.0

# Choc sur la carrosserie : le comma est colle au pare-brise, il sent ce que la voiture
# sent. Mesure du 22/09/2026, voiture garee, 9480 echantillons a 105 Hz : la norme de
# l'acceleration s'ecarte au pire de 0,019 m/s2 de sa valeur de repos. Le seuil est 25
# fois plus haut, et il faut deux echantillons pour ecarter un sursaut isole du capteur.
DEFAULT_SHOCK_MS2 = 0.5


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


def mark_running() -> None:
  """Pose pendant la veille, retire a l'arret volontaire : s'il est encore la au
  demarrage suivant, c'est que la veille s'est arretee net, courant coupe."""
  try:
    with open(RUNNING_PATH, "w") as f:
      f.write("1")
  except OSError:
    pass


def clear_running() -> None:
  try:
    os.unlink(RUNNING_PATH)
  except OSError:
    pass


def was_running() -> bool:
  return os.path.exists(RUNNING_PATH)


def auto_enabled() -> bool:
  return os.path.exists(AUTO_PATH)


def set_auto(enabled: bool) -> None:
  """Armement automatique a l'arret du moteur, reglable depuis l'ecran."""
  if enabled:
    try:
      with open(AUTO_PATH, "w") as f:
        f.write("1")
    except OSError:
      pass
  else:
    try:
      os.unlink(AUTO_PATH)
    except OSError:
      pass


def should_auto_arm(ignition: bool, ignition_prev: bool) -> bool:
  """Vrai au front descendant du contact : le moteur vient d'etre coupe.

  On n'arme que sur la transition, jamais tant que la voiture reste a l'arret : si la
  veille s'est arretee d'elle-meme, batterie basse par exemple, elle ne doit pas
  repartir en boucle. Il faudra un nouveau cycle de contact.
  """
  return auto_enabled() and ignition_prev and not ignition and not is_armed()


def _read_number(path: str, default, cast):
  try:
    with open(path) as f:
      return cast(f.read().strip())
  except (OSError, ValueError):
    return default


def motion_delta() -> int:
  """Reglable a chaud : echo 20 > /data/sentry_motion_delta"""
  return _read_number(MOTION_DELTA_PATH, DEFAULT_MOTION_DELTA, int)


def max_bytes() -> int:
  """Reglable a chaud : echo 4 > /data/sentry_max_gb"""
  return int(_read_number(MAX_GB_PATH, DEFAULT_MAX_GB, float) * 1024 ** 3)


def shock_threshold() -> float:
  """Reglable a chaud : echo 0.3 > /data/sentry_shock_ms2"""
  return _read_number(SHOCK_PATH, DEFAULT_SHOCK_MS2, float)


def _read_camera_number(path: str, cam: str | None, default, cast):
  """Le fichier propre a la camera l'emporte, puis le fichier commun, puis le defaut."""
  if cam:
    value = _read_number(f"{path}_{cam}", None, cast)
    if value is not None:
      return value
  return _read_number(path, default, cast)


def motion_area(cam: str | None = None) -> float:
  """Reglable a chaud : echo 0.02 > /data/sentry_motion_area_d (ou sans suffixe pour toutes)"""
  default = CAMERA_MOTION.get(cam, (DEFAULT_MOTION_AREA, DEFAULT_MOTION_HOLD))[0]
  return _read_camera_number(MOTION_AREA_PATH, cam, default, float)


def motion_hold(cam: str | None = None) -> int:
  """Images consecutives exigees. Reglable a chaud : echo 2 > /data/sentry_motion_hold_d"""
  default = CAMERA_MOTION.get(cam, (DEFAULT_MOTION_AREA, DEFAULT_MOTION_HOLD))[1]
  return max(1, _read_camera_number(MOTION_HOLD_PATH, cam, default, int))
