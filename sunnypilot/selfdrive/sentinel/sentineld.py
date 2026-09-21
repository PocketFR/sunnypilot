#!/usr/bin/env python3
"""
Mode sentinelle : surveille la voiture garee et n'enregistre que ce qui se passe.

Arme depuis le panneau « sentinelle » (parametre SentryMode), le service tourne hors
route aux cotes de camerad, qui est demarre par la meme porte. Deux declencheurs :
le mouvement vu par les trois cameras, celle de l'habitacle comprise puisqu'elle couvre
les cotes, et toute trame CAN entendue par le panda. Pendant un evenement, une image par seconde et par camera est ecrite en JPEG,
precedee des dernieres images gardees en memoire, de quoi voir ce qui s'est passe juste
avant. Le NAS en fait ensuite un timelapse.

La sentinelle se desarme seule au demarrage du moteur, et s'arrete si la tension 12 V
descend sous SentryMinVoltage : hors contact, la batterie ne se recharge pas.

Le nom « sentinel » evite la collision avec system/sentry.py, qui est le rapport de
crash Sentry.io.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import signal
import time
from collections import deque

import numpy as np

from openpilot.sunnypilot.selfdrive.common.boot_time import (TIME_FORMAT, TIME_SLACK, boot_id,
                                                             clock_is_set, real_time)
from openpilot.sunnypilot.selfdrive.sentinel import state
from openpilot.sunnypilot.selfdrive.sentinel.state import write_status

SENTRY_DIR = "/data/media/0/sentry"
# Les evenements sont ranges par jour : <SENTRY_DIR>/<AAAAMMJJ>/<AAAAMMJJ_HHMMSS>. Sur
# plusieurs nuits de veille la liste plate devenait illisible, dans le lecteur comme sur
# le NAS. Les evenements plats d'avant ce classement restent lus (voir event_dirs).
DAY_FORMAT = "%Y%m%d"
EVENT_FORMAT = "%Y%m%d_%H%M%S"
DAY_RE = re.compile(r"\d{8}$")

# Cameras : prefixe de fichier -> nom du flux VisionIPC
CAMERAS = {"f": "VISION_STREAM_ROAD", "e": "VISION_STREAM_WIDE_ROAD", "d": "VISION_STREAM_DRIVER"}
# Les trois cameras declenchent : celle de l'habitacle regarde par les vitres laterales,
# donc elle voit venir ce que les deux autres, tournees vers l'avant, manquent.
MOTION_CAMERAS = tuple(CAMERAS)

FRAME_INTERVAL = 1.0      # une image par seconde et par camera
PRE_ROLL = 10             # images gardees en memoire avant le declencheur
POST_EVENT_S = 20.        # on continue a enregistrer apres le dernier declencheur
MAX_BYTES = 2 * 1024 ** 3  # place reservee aux evenements
MIN_FREE_PERCENT = 10.    # on n'ecrit plus en dessous, les trajets passent avant
LOW_VOLTAGE_S = 60.       # duree sous le seuil avant de s'arreter
JPEG_QUALITY = 80

# Releve de consommation : l'appareil mesure lui-meme la puissance tiree de la voiture
# (/sys/class/hwmon/hwmon1/power1_input). Une ligne toutes les 30 s suffit a tracer une
# nuit, et permet de dimensionner une batterie auxiliaire sur des chiffres.
TRACE_PATH = "/data/sentry_power.csv"
TRACE_INTERVAL = 30.
TRACE_MAX_BYTES = 512 * 1024
TRACE_KEEP_LINES = 2000
TRACE_HEADER = "time,mono,boot,voltage_mv,power_w,som_w,recording,events\n"

# Mouvement : pas de sous-echantillonnage. L'ecart et la surface sont dans state.py,
# reglables par fichier sans toucher au code.
MOTION_STEP = 16


class MotionDetector:
  """Compare deux images sur le plan de luminance seul, sous-echantillonne.

  Deux filtres, chacun contre une fausse alerte precise :

  - l'ecart median est retire avant le seuillage, car un nuage qui passe ou un lampadaire
    qui s'allume decalent toute l'image d'un bloc ;
  - seules les cellules formant un carre de 2x2 sont comptees, car le vent dans les arbres
    fait du grain disperse la ou une personne fait une tache. Mesure sur des evenements
    reels : ce filtre fait passer le rapport entre les deux de 5 a 33.
  """

  def __init__(self, step: int = MOTION_STEP, delta: int | None = None, area: float | None = None):
    self.step = step
    self.delta = state.motion_delta() if delta is None else delta
    self.area = state.motion_area() if area is None else area
    self.previous: np.ndarray | None = None

  def score(self, y: np.ndarray) -> float:
    """Part des cellules en mouvement groupe, entre 0 et 1. -1 sans image precedente."""
    cells = y[::self.step, ::self.step].astype(np.int16)
    previous, self.previous = self.previous, cells
    if previous is None or previous.shape != cells.shape:
      return -1.

    diff = cells - previous
    moved = np.abs(diff - np.median(diff)) > self.delta
    if moved.shape[0] < 2 or moved.shape[1] < 2:
      return float(moved.mean())
    grouped = moved[:-1, :-1] & moved[1:, :-1] & moved[:-1, 1:] & moved[1:, 1:]
    return float(grouped.mean())

  def update(self, y: np.ndarray) -> bool:
    return self.score(y) > self.area


def extract_y(buf) -> np.ndarray:
  """Plan de luminance seul : suffit au mouvement, et ne coute presque rien (0,2 ms)."""
  return np.array(buf.data[:buf.uv_offset], dtype=np.uint8).reshape((-1, buf.stride))[:buf.height, :buf.width]


def nv12_to_ycbcr(buf) -> np.ndarray:
  """NV12 vers YCbCr pleine resolution, en reprenant le decoupage de snapshot.py.

  On s'arrete a YCbCr : le JPEG est nativement dans cet espace, passer par le RGB
  revenait a faire deux conversions pour rien. Mesure sur l'appareil, image 1344x760 :
  484 ms par le RGB, 47 ms ici, pour la meme image.
  """
  uv_height = ((buf.height // 2) + 15) // 16 * 16
  uv = buf.data[buf.uv_offset:buf.uv_offset + buf.stride * uv_height]

  y = extract_y(buf)
  u = np.array(uv[::2], dtype=np.uint8).reshape((-1, buf.stride // 2))[:buf.height // 2, :buf.width // 2]
  v = np.array(uv[1::2], dtype=np.uint8).reshape((-1, buf.stride // 2))[:buf.height // 2, :buf.width // 2]

  ul = np.repeat(np.repeat(u, 2, axis=1), 2, axis=0)[:y.shape[0], :y.shape[1]]
  vl = np.repeat(np.repeat(v, 2, axis=1), 2, axis=0)[:y.shape[0], :y.shape[1]]
  return np.dstack((y, ul, vl))


def encode_jpeg(buf, quality: int = JPEG_QUALITY) -> bytes:
  from io import BytesIO

  from PIL import Image

  out = BytesIO()
  Image.fromarray(nv12_to_ycbcr(buf), mode="YCbCr").save(out, "JPEG", quality=quality)
  return out.getvalue()


def dir_size(path: str) -> int:
  total = 0
  for root, _, files in os.walk(path):
    for name in files:
      try:
        total += os.path.getsize(os.path.join(root, name))
      except OSError:
        continue
  return total


class EventRecorder:
  """Ouvre un dossier par evenement, y deverse la pre-memoire puis les images suivantes."""

  def __init__(self, root: str = SENTRY_DIR, pre_roll: int = PRE_ROLL, post_event_s: float = POST_EVENT_S):
    self.root, self.post_event_s = root, post_event_s
    self.buffers: dict[str, deque] = {cam: deque(maxlen=pre_roll) for cam in CAMERAS}
    self.path: str | None = None
    self.index = 0
    self.last_trigger = 0.
    self.triggers: list[str] = []
    self.details: dict = {}
    self.frames = 0
    self.started_at = self.started_mono = 0.

  @property
  def recording(self) -> bool:
    return self.path is not None

  def offer(self, cam: str, jpeg: bytes) -> None:
    """Chaque image passe par la : ecrite si un evenement est ouvert, gardee sinon.

    Une image par camera et par tour de boucle, puis tick() : les trois cameras partagent
    le meme numero d'image, ce qui permet de les recoller au montage. Deux images offertes
    pour la meme camera sans tick() intercale ecrasent le meme fichier.
    """
    if self.recording:
      self._write(cam, jpeg, self.index)
    else:
      self.buffers[cam].append(jpeg)

  def trigger(self, kind: str, now: float, meta: dict | None = None) -> None:
    self.last_trigger = now
    if kind not in self.triggers:
      self.triggers.append(kind)
    self._merge(meta or {})
    if not self.recording:
      self._open(now)
    else:
      self._write_meta(self._meta())   # une camera peut s'ajouter en cours d'evenement

  def _merge(self, meta: dict) -> None:
    """Le detail s'accumule : une deuxieme camera qui voit bouger s'ajoute a la premiere,
    sans doublon, plutot que de remplacer ce qui etait deja note."""
    for key, value in meta.items():
      if isinstance(value, list):
        known = self.details.setdefault(key, [])
        known.extend(v for v in value if v not in known)
      else:
        self.details[key] = value

  def _meta(self) -> dict:
    meta = dict(self.details)
    meta.update({"time": self.started_at, "mono": self.started_mono, "boot_id": boot_id(),
                 "triggers": self.triggers, "frames": self.frames})
    return meta

  def tick(self, now: float) -> None:
    """Avance d'une image, et ferme l'evenement passe le delai sans declencheur."""
    if not self.recording:
      return
    self.index += 1
    if now - self.last_trigger > self.post_event_s:
      self.close()

  def _open(self, now: float) -> None:
    self.started_at, self.started_mono = time.time(), time.monotonic()
    self.path = event_path(self.root, self.started_at)
    os.makedirs(self.path, exist_ok=True)  # cree aussi le dossier du jour
    self.index, self.frames = 0, 0
    self._write_meta(self._meta())

    # la pre-memoire part en indices negatifs : ce qui precede l'evenement
    for cam, buffered in self.buffers.items():
      for offset, jpeg in enumerate(buffered):
        self._write(cam, jpeg, offset - len(buffered))
      buffered.clear()

  def _write(self, cam: str, jpeg: bytes, index: int) -> None:
    assert self.path is not None
    name = "%s_%s%03d.jpg" % (cam, "m" if index < 0 else "", abs(index))
    try:
      with open(os.path.join(self.path, name), "wb") as f:
        f.write(jpeg)
      self.frames += 1
    except OSError:
      pass

  def _write_meta(self, meta: dict) -> None:
    assert self.path is not None
    try:
      with open(os.path.join(self.path, "meta.json"), "w") as f:
        json.dump(meta, f)
    except OSError:
      pass

  def close(self) -> str | None:
    if not self.recording:
      return None
    path = self.path
    meta = self._meta()
    meta["closed"] = time.time()
    self._write_meta(meta)
    self.path, self.triggers, self.details, self.index = None, [], {}, 0
    return path


def append_trace(row: dict, path: str = TRACE_PATH) -> None:
  """Ajoute une ligne au releve, et borne le fichier comme le journal du diagnostic.

  L'heure murale est fausse au reveil, d'ou mono et boot : l'analyse recale ensuite
  (voir sunnypilot/selfdrive/common/boot_time.py).
  """
  try:
    new_file = not os.path.exists(path)
    with open(path, "a") as f:
      if new_file:
        f.write(TRACE_HEADER)
      f.write("%.0f,%.1f,%s,%d,%.2f,%.2f,%d,%d\n" % (
        row["time"], row["mono"], row["boot"], row["voltage_mv"],
        row["power_w"], row["som_w"], row["recording"], row["events"]))
  except OSError:
    return

  try:
    if os.path.getsize(path) <= TRACE_MAX_BYTES:
      return
    with open(path) as f:
      lines = f.readlines()
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
      f.write(TRACE_HEADER)
      f.writelines(lines[-TRACE_KEEP_LINES:])
    os.replace(tmp, path)
  except OSError:
    pass


def read_json(path: str) -> dict:
  try:
    with open(path) as f:
      return json.load(f)
  except (OSError, ValueError):
    return {}


def event_path(root: str, when: float) -> str:
  """Chemin d'un evenement : <root>/<AAAAMMJJ>/<AAAAMMJJ_HHMMSS>."""
  stamp = time.localtime(when)
  return os.path.join(root, time.strftime(DAY_FORMAT, stamp), time.strftime(EVENT_FORMAT, stamp))


def event_dirs(root: str = SENTRY_DIR) -> list[str]:
  """Les evenements, du plus ancien au plus recent.

  Un dossier dont le nom est une date seule (AAAAMMJJ) est un dossier de jour : ses enfants
  sont les evenements. Les autres sont des evenements ranges a plat, d'avant ce classement.
  """
  try:
    entries = sorted(os.listdir(root))
  except OSError:
    return []

  dirs = []
  for name in entries:
    path = os.path.join(root, name)
    if not os.path.isdir(path):
      continue
    if DAY_RE.fullmatch(name):
      try:
        dirs += [os.path.join(path, d) for d in sorted(os.listdir(path))
                 if os.path.isdir(os.path.join(path, d))]
      except OSError:
        continue
    else:
      dirs.append(path)
  return sorted(dirs, key=lambda d: os.path.getmtime(d))


def prune_empty_days(root: str = SENTRY_DIR) -> None:
  """Retire les dossiers de jour restes vides apres une purge ou un recalage."""
  try:
    entries = sorted(os.listdir(root))
  except OSError:
    return
  for name in entries:
    path = os.path.join(root, name)
    if DAY_RE.fullmatch(name) and os.path.isdir(path):
      try:
        os.rmdir(path)  # echoue si le dossier n'est pas vide, c'est voulu
      except OSError:
        pass


def prune(root: str = SENTRY_DIR, max_bytes: int = MAX_BYTES, keep: str | None = None) -> list[str]:
  """Supprime les evenements les plus anciens au-dela du quota. Ne touche qu'a SENTRY_DIR,
  jamais aux trajets : ceux-la ont leur propre nettoyage (system/loggerd/deleter.py)."""
  dirs = event_dirs(root)
  sizes = {d: dir_size(d) for d in dirs}
  total = sum(sizes.values())
  removed = []
  for d in dirs:
    if total <= max_bytes:
      break
    if d == keep:
      continue
    try:
      shutil.rmtree(d)
    except OSError:
      continue
    total -= sizes[d]
    removed.append(d)
  if removed:
    prune_empty_days(root)
  return removed


def recalibrate_events(root: str = SENTRY_DIR) -> list[tuple[str, str]]:
  """Renomme les evenements dates par une horloge pas encore recalee.

  Le nom du dossier vient de l'heure murale, fausse au reveil. meta.json porte le temps
  monotone et l'identifiant de demarrage : tant qu'on est dans le meme demarrage, on
  retrouve l'heure reelle (voir sunnypilot/selfdrive/common/boot_time.py).
  """
  if not clock_is_set():
    return []

  renamed = []
  current_boot = boot_id()
  for path in event_dirs(root):
    meta = read_json(os.path.join(path, "meta.json"))
    mono, boot = meta.get("mono"), meta.get("boot_id")
    if mono is None or boot != current_boot:
      continue

    corrected = real_time(mono)
    if abs(corrected - meta.get("time", 0)) <= TIME_SLACK:
      continue

    target = event_path(root, corrected)
    if os.path.exists(target):
      continue
    try:
      os.makedirs(os.path.dirname(target), exist_ok=True)
      os.rename(path, target)
      os.utime(target, (corrected, corrected))  # l'ordre de purge suit la date des dossiers
    except OSError:
      continue
    meta["time"] = corrected
    try:
      with open(os.path.join(target, "meta.json"), "w") as f:
        json.dump(meta, f)
    except OSError:
      pass
    renamed.append((path, target))
  if renamed:
    prune_empty_days(root)  # le jour d'origine peut s'etre vide
  return renamed


def last_event_time(root: str = SENTRY_DIR) -> float | None:
  dirs = event_dirs(root)
  if not dirs:
    return None
  meta = read_json(os.path.join(dirs[-1], "meta.json"))
  return meta.get("time") or os.path.getmtime(dirs[-1])


class VoltageWatchdog:
  """S'arreter avant d'avoir vide la batterie : hors contact, elle ne se recharge pas."""

  def __init__(self, minimum_mv: int, duration: float = LOW_VOLTAGE_S):
    self.minimum_mv, self.duration = minimum_mv, duration
    self.since: float | None = None

  def update(self, voltage_mv: float, now: float) -> bool:
    """Vrai quand la tension est restee trop basse assez longtemps."""
    if not voltage_mv or voltage_mv >= self.minimum_mv:
      self.since = None
      return False
    if self.since is None:
      self.since = now
    return (now - self.since) >= self.duration


class Sentry:
  """Etat de la surveillance, sans dependance aux cameras ni au CAN : les images et les
  trames lui sont donnees par la boucle principale, ce qui permet de la tester seule."""

  def __init__(self, root: str = SENTRY_DIR):
    self.recorder = EventRecorder(root)
    self.motion = {cam: MotionDetector() for cam in MOTION_CAMERAS}
    self.watchdog = VoltageWatchdog(state.min_voltage_mv())
    self.root = root
    self.voltage_mv = 0.
    self.current_ma = 0.
    self.power_w = 0.
    self.som_w = 0.
    self.last_frame = 0.
    self.stop_reason: str | None = None
    self.disk_full = False

  def armed(self) -> bool:
    return state.is_armed()

  def check_stop(self, ignition: bool, voltage_mv: float, now: float) -> str | None:
    """Trois facons de s'arreter : le moteur demarre, la batterie faiblit, ou l'ecran desarme."""
    self.voltage_mv = voltage_mv
    if ignition:
      self.stop_reason = "ignition"
    elif self.watchdog.update(voltage_mv, now):
      self.stop_reason = "voltage"
    elif not self.armed():
      self.stop_reason = "disarmed"
    return self.stop_reason

  def on_boot(self, now: float, power_cut: bool) -> None:
    """Un demarrage alors que la sentinelle veillait merite d'etre filme.

    Debrancher le comma est la premiere chose a faire pour le faire taire : au
    rebranchement il repart en veille, et enregistre aussitot celui qui est devant.
    power_cut distingue la coupure franche du redemarrage voulu a l'armement.
    """
    self.recorder.trigger("boot", now, {"power_cut": power_cut})

  def on_can(self, now: float, buses: set[int]) -> None:
    """Hors contact, une trame sur le bus est deja un evenement : reveil du reseau,
    ouverture, alarme. Le detail des identifiants viendra plus tard si besoin."""
    self.recorder.trigger("can", now, {"can_buses": sorted(buses)})

  def on_frames(self, frames: dict, now: float) -> None:
    """frames : prefixe de camera -> (plan Y, fonction rendant le JPEG)."""
    for cam, (y, _) in frames.items():
      if cam in self.motion and self.motion[cam].update(y):
        self.recorder.trigger("motion", now, {"motion_cameras": [cam]})

    if self.disk_full:
      return
    for cam, (_, jpeg) in frames.items():
      self.recorder.offer(cam, jpeg())
    self.recorder.tick(now)
    self.last_frame = now

  def trace_row(self) -> dict:
    """Une ligne de releve : ce que la voiture fournit, ce que l'appareil consomme."""
    return {"time": time.time(), "mono": time.monotonic(), "boot": boot_id()[:8],
            "voltage_mv": self.voltage_mv, "power_w": self.power_w, "som_w": self.som_w,
            "recording": int(self.recorder.recording), "events": len(event_dirs(self.root))}

  def status(self) -> dict:
    return {"armed": self.armed(), "recording": self.recorder.recording,
            "power_w": self.power_w, "som_w": self.som_w,
            "triggers": list(self.recorder.triggers),
            "motion_cameras": list(self.recorder.details.get("motion_cameras") or []), "voltage_mv": self.voltage_mv,
            "min_voltage_mv": self.watchdog.minimum_mv, "bytes": dir_size(self.root),
            "events": len(event_dirs(self.root)), "last_event": last_event_time(self.root),
            "disk_full": self.disk_full, "stop_reason": self.stop_reason,
            "time": time.time(), "mono": time.monotonic(), "boot_id": boot_id()}


def finish(sentry) -> bool:
  """Fermeture propre. Retourne True s'il faut eteindre l'appareil.

  Le marqueur est retire ici : un arret ordonne, redemarrage ou extinction, ne doit pas
  passer pour une coupure de courant au demarrage suivant. Seule une veille interrompue
  net laisse le marqueur derriere elle.
  """
  sentry.recorder.close()
  state.clear_running()
  if sentry.stop_reason in ("ignition", "voltage"):
    state.disarm()
  write_status(sentry.status())
  return sentry.stop_reason == "voltage"


def main() -> None:
  import cereal.messaging as messaging
  from msgq.visionipc import VisionIpcClient, VisionStreamType

  from openpilot.common.params import Params
  from openpilot.common.realtime import Ratekeeper
  from openpilot.system.hardware import HARDWARE
  from openpilot.system.loggerd.config import get_available_percent

  def screen_off() -> None:
    """A reaffirmer, pas a poser une fois : quand l'armement se fait sans redemarrage,
    l'interface est encore en train de s'arreter et rallume le retroeclairage apres nous."""
    try:
      HARDWARE.set_screen_brightness(0)
    except Exception:
      pass

  screen_off()

  def read_power(voltage_mv: float, current_ma: float) -> tuple[float, float]:
    """Puissance tiree de la voiture, et part du seul calculateur.

    Le capteur que lit openpilot (/sys/class/hwmon/hwmon1/power1_input) n'existe pas sur
    mici et rend 0 : on calcule donc la puissance cote voiture a partir de la tension et
    du courant rapportes par le panda, verifies a 2,56 W veille armee contre 2,05 W au
    repos. La difference avec la part du calculateur donne le cout des cameras.
    """
    try:
      som = HARDWARE.get_som_power_draw() or 0.
    except Exception:
      som = 0.
    try:
      total = HARDWARE.get_current_power_draw() or 0.
    except Exception:
      total = 0.
    if not total:
      total = voltage_mv * current_ma / 1e6
    return total, som

  sentry = Sentry()
  # marqueur laisse en place par la veille precedente = elle ne s'est pas arretee d'elle-meme
  power_cut = state.was_running()
  state.mark_running()
  sentry.on_boot(time.monotonic(), power_cut)

  sm = messaging.SubMaster(["can", "pandaStates", "peripheralState"])
  clients = {cam: VisionIpcClient("camerad", getattr(VisionStreamType, stream), True)
             for cam, stream in CAMERAS.items()}

  stopping = False

  def on_signal(*_):
    """Le manager arrete le service par un signal : on sort de la boucle pour passer
    par la fermeture propre, au lieu de mourir en laissant le marqueur."""
    nonlocal stopping
    stopping = True

  signal.signal(signal.SIGINT, on_signal)
  signal.signal(signal.SIGTERM, on_signal)

  recalibrated = False
  last_status = last_prune = last_trace = 0.
  rk = Ratekeeper(2.0, print_delay_threshold=None)

  while not stopping:
    now = time.monotonic()
    sm.update(0)

    ignition = any(ps.ignitionLine or ps.ignitionCan for ps in sm["pandaStates"]) if sm.updated["pandaStates"] else False
    if sm.updated["peripheralState"]:
      voltage, sentry.current_ma = sm["peripheralState"].voltage, sm["peripheralState"].current
    else:
      voltage = sentry.voltage_mv
    if sentry.check_stop(ignition, voltage, now):
      break

    if sm.updated["can"] and len(sm["can"]):
      sentry.on_can(now, {c.src for c in sm["can"]})

    if now - sentry.last_frame >= FRAME_INTERVAL:
      sentry.disk_full = get_available_percent(100.) < MIN_FREE_PERCENT
      frames = {}
      for cam, client in clients.items():
        if not client.is_connected() and not client.connect(False):
          continue
        buf = client.recv(20)
        if buf is None or len(buf.data) == 0:   # buf.data est un memoryview sur l'appareil
          continue
        frames[cam] = (extract_y(buf), lambda b=buf: encode_jpeg(b))
      if frames:
        sentry.on_frames(frames, now)

    if now - last_prune > 60.:
      prune(sentry.root, keep=sentry.recorder.path)
      last_prune = now
      if not recalibrated:
        recalibrated = bool(recalibrate_events(sentry.root)) or clock_is_set()

    if now - last_trace > TRACE_INTERVAL:
      sentry.power_w, sentry.som_w = read_power(sentry.voltage_mv, sentry.current_ma)
      append_trace(sentry.trace_row())
      screen_off()
      last_trace = now

    if now - last_status > 5.:
      write_status(sentry.status())
      screen_off()
      last_status = now

    rk.keep_time()

  if finish(sentry):
    # hors contact la batterie ne se recharge pas : on rend la main avant de la vider
    Params().put_bool("DoShutdown", True)


if __name__ == "__main__":
  main()
