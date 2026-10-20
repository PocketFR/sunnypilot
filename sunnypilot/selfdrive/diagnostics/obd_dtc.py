#!/usr/bin/env python3
"""
Lecture et effacement des codes defaut depuis le comma, et remise en service du SCC.

Chemin : prise OBD -> comma power -> panda, bus 1 avec le multiplexage OBD actif.
Mesure sur la voiture : le bus 0 (C-CAN du harness camera) ne repond pas en
diagnostic ; 0x7E0 et 0x7E1 repondent sur le bus 1 une fois set_obd(True) pose.

Le panda n'accepte qu'un seul maitre, et interroger un calculateur impose de le
basculer en mode ELM327, ce qui lui retire sa capacite a commander la voiture.
Tout se fait donc au demarrage du manager, avant que pandad ne prenne le panda :
le contact vient d'etre mis, la voiture est a l'arret, le panda est libre.

L'interface (panneau Diagnostics) ne fait que lire le resultat, et au besoin
poser un drapeau puis relancer openpilot.

Remise en service du SCC : pour piloter les freins, openpilot baillonne le
calculateur SCC (UDS 0x28 « desactive emission et reception »), maintenu par un
tester present chaque seconde, et ne le reactive jamais. A chaque redemarrage,
ni le SCC ni openpilot ne parlent a l'ESP pendant quelques secondes, d'ou les
codes C1638 (ACC communication error) et C16B8. On envoie donc ici la commande
inverse, au plus tot dans le cycle : c'est le seul moment ou elle passe, car en
mode de securite Hyundai le panda ne laisse passer que le tester present sur
0x7D0 (opendbc/safety/modes/hyundai.h).

Interrupteur : creer /data/dtc_disable desactive completement la lecture au boot.
"""
from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import threading
import time

from openpilot.sunnypilot.selfdrive.common.boot_time import (TIME_FORMAT, TIME_SLACK,
                                                             boot_id as _boot_id,
                                                             clock_is_set as _clock_is_set,
                                                             real_time as _real_time)

STATUS_PATH = "/data/dtc_status.json"
REQUEST_PATH = "/data/dtc_request"
LOG_PATH = "/data/dtc_log.txt"
DISABLE_PATH = "/data/dtc_disable"

ENGINE_ECU = 0x7E0
SCC_ECU = 0x7D0
ABS_ECU = 0x7D1
EPS_ECU = 0x7D4
BUS = 1
# Calculateurs qui repondent en diagnostic sur cette voiture (scan sur bus 1, OBD actif).
OTHER_ECUS = {0x7E1: "trans", SCC_ECU: "scc", ABS_ECU: "abs", EPS_ECU: "eps",
              0x7C6: "cluster", 0x7B7: "corner radar", 0x7B3: "0x7B3"}
# Les codes en C du SCC, de l'ABS et de la direction assistee sont provoques par openpilot
# lui-meme, qui baillonne le SCC : ces modules journalisent la perte de communication
# (C1638, C16B8 pour les deux premiers, C1804 pour la direction). L'effacement les vise
# donc avec le moteur. Les autres calculateurs sont affiches mais jamais effaces.
CHASSIS_ECUS = {SCC_ECU: "scc", ABS_ECU: "abs", EPS_ECU: "eps"}
CLEAR_ECUS = {ENGINE_ECU: "moteur", **CHASSIS_ECUS}
# Odometre : combine (0x7C6), DID 0xB002, octets 6-8 en gros-boutiste, en km.
# Verifie contre le tableau de bord : 131404.
CLUSTER_ECU = 0x7C6
ODO_DID = 0xB002
ODO_OFFSET = 6

# Les modes 01 et 02 partagent le meme jeu de PID : la valeur vivante et la valeur figee a
# l'instant du defaut se demandent et se decodent de la meme facon (SAE J1979). Le jeu est
# choisi pour un defaut de melange : les deux corrections de carburant disent ou chercher,
# le reste donne les conditions dans lesquelles elles ont ete relevees.
#   (PID, nom, octets attendus, decodage)
PIDS = (
  (0x04, "charge_pct", 1, lambda d: round(d[0] * 100 / 255, 1)),
  (0x05, "liquide_c", 1, lambda d: d[0] - 40),
  (0x06, "trim_court_pct", 1, lambda d: round((d[0] - 128) * 100 / 128, 1)),
  (0x07, "trim_long_pct", 1, lambda d: round((d[0] - 128) * 100 / 128, 1)),
  (0x0B, "collecteur_kpa", 1, lambda d: d[0]),
  (0x0C, "regime_tr", 2, lambda d: round(((d[0] << 8) | d[1]) / 4)),
  (0x0D, "vitesse_kmh", 1, lambda d: d[0]),
  (0x0F, "air_admis_c", 1, lambda d: d[0] - 40),
  (0x10, "debit_air_gs", 2, lambda d: round(((d[0] << 8) | d[1]) / 100, 2)),
  (0x11, "papillon_pct", 1, lambda d: round(d[0] * 100 / 255, 1)),
)
SCAN_TIMEOUT = 30.
# Au contact, le budget borne le temps pendant lequel openpilot n'est pas pret : une lecture
# complete prend quelques secondes, quinze laissent de la marge sans faire attendre un trajet.
IGNITION_SCAN_TIMEOUT = 15.
HANDOVER_DELAY = 1.0
LETTERS = "PCBU"
# "2026-03-24 14:46:14 (t+31s 3f2a1c9d) message"
LOG_LINE = re.compile(r"^(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d) \(t\+(\d+)s ([0-9a-f]{8})\) (.*)$")
# attente de la mise a l'heure : le wifi ou le GPS mettent de quelques secondes a
# quelques minutes, au-dela on abandonne plutot que de veiller pour rien
CLOCK_POLL = 20.
CLOCK_WAIT = 30 * 60.
# Le journal vit sur /data et grossit de deux ou trois lignes par mise du contact.
# Une ligne fait ~200 octets, donc ce plafond garde environ un an d'historique.
LOG_MAX_BYTES = 128 * 1024
LOG_KEEP_LINES = 400


def dtc_str(b0: int, b1: int) -> str:
  return "%s%d%X%02X" % (LETTERS[(b0 >> 6) & 3], (b0 >> 4) & 3, b0 & 0xF, b1)


def _fmt(valeurs: dict) -> str:
  """Un dictionnaire de mesures en une ligne lisible, dans l'ordre de la table PIDS."""
  ordre = ["code"] + [nom for _, nom, _, _ in PIDS]
  return " ".join("%s=%s" % (k, valeurs[k]) for k in ordre if k in valeurs) or "-"


def _log(msg: str) -> None:
  """Horodate aussi en secondes depuis le demarrage et par debut d'identifiant de
  demarrage : au boot l'horloge murale est encore a la date par defaut d'AGNOS, et
  ces deux repères permettent de recaler la ligne apres coup (voir _fix_log_times)."""
  try:
    with open(LOG_PATH, "a") as f:
      f.write("%s (t+%ds %s) %s\n" % (time.strftime(TIME_FORMAT), time.monotonic(), _boot_id()[:8], msg))
  except OSError:
    pass
  _trim_log()


def _trim_log() -> None:
  """Garde le journal borne : il n'a pas a grossir indefiniment sur /data. Au-dela du
  plafond, on ne conserve que les LOG_KEEP_LINES dernieres lignes, les plus recentes."""
  try:
    if os.path.getsize(LOG_PATH) <= LOG_MAX_BYTES:
      return
    with open(LOG_PATH) as f:
      lines = f.readlines()
    tmp = LOG_PATH + ".tmp"
    with open(tmp, "w") as f:
      f.writelines(lines[-LOG_KEEP_LINES:])
    os.replace(tmp, LOG_PATH)
  except OSError:
    pass


# ---------------------------------------------------------------- cote interface

def read_status() -> dict:
  """Dernier releve. N'importe rien de lourd : appele depuis le process UI."""
  _fix_log_times()
  try:
    with open(STATUS_PATH) as f:
      return _with_corrected_time(json.load(f))
  except (OSError, ValueError):
    return {}


def _with_corrected_time(status: dict) -> dict:
  """Recale l'horodatage du releve.

  Au demarrage, le comma n'a ni GPS ni wifi : son horloge est encore a la date par
  defaut d'AGNOS, et le releve est donc horodate n'importe quand. Le temps monotone,
  lui, est juste. Tant qu'on est dans le meme demarrage, on retrouve l'heure reelle
  du releve des que l'horloge est recalee. On reecrit alors le fichier, pour que les
  demarrages suivants lisent la bonne heure.
  """
  mono, boot = status.get("mono"), status.get("boot_id")
  if mono is None or not boot or boot != _boot_id() or not _clock_is_set():
    return status

  corrected = _real_time(mono)
  if abs(corrected - status.get("time", 0)) > TIME_SLACK:
    status["time"] = corrected
    try:
      with open(STATUS_PATH, "w") as f:
        json.dump(status, f)
    except OSError:
      pass
  return status


def _recalibrate() -> None:
  """Recale ce qui a ete horodate avant la mise a l'heure : le releve et le journal."""
  _fix_log_times()
  try:
    with open(STATUS_PATH) as f:
      _with_corrected_time(json.load(f))  # reecrit le fichier s'il y a de quoi corriger
  except (OSError, ValueError):
    pass


def _fix_log_times() -> None:
  """Recale les lignes du journal ecrites avant que l'horloge ne soit a l'heure.

  Meme principe que pour le releve, mais ligne par ligne : chaque ligne porte ses
  secondes depuis le demarrage et le debut de l'identifiant de demarrage. Seules les
  lignes du demarrage courant sont recalees, les autres ne sont plus rattrapables et
  restent telles quelles, comme les lignes ecrites avant ce format.
  """
  if not _clock_is_set():
    return

  boot = _boot_id()[:8]
  try:
    with open(LOG_PATH) as f:
      lines = f.readlines()
  except OSError:
    return

  fixed, changed = [], False
  for line in lines:
    m = LOG_LINE.match(line.rstrip("\n"))
    if m is None or m.group(3) != boot:
      fixed.append(line)
      continue

    stamp, mono, rest = m.group(1), float(m.group(2)), m.group(4)
    real = _real_time(mono)
    try:
      written = time.mktime(time.strptime(stamp, TIME_FORMAT))
    except ValueError:
      fixed.append(line)
      continue

    if abs(real - written) <= TIME_SLACK:
      fixed.append(line)
      continue

    fixed.append("%s (t+%ds %s) %s\n" % (time.strftime(TIME_FORMAT, time.localtime(real)), mono, boot, rest))
    changed = True

  if not changed:
    return
  try:
    tmp = LOG_PATH + ".tmp"
    with open(tmp, "w") as f:
      f.writelines(fixed)
    os.replace(tmp, LOG_PATH)
  except OSError:
    pass


def pending_request() -> bool:
  """Une demande a ete faite depuis l'ecran et n'a pas encore ete traitee."""
  return os.path.exists(REQUEST_PATH)


def request(action: str) -> None:
  """'read' ou 'clear', traite au prochain demarrage du manager ou au prochain contact."""
  try:
    with open(REQUEST_PATH, "w") as f:
      f.write(action)
  except OSError:
    pass


# ---------------------------------------------------------------- cote demarrage

def fix_log_when_clock_is_set(poll: float = CLOCK_POLL, wait: float = CLOCK_WAIT) -> threading.Thread | None:
  """Reprend le journal des que l'horloge est recalee, sans rien demander a personne.

  Au demarrage l'horloge vaut la date de l'image AGNOS, et la mise a l'heure arrive
  plus tard, par le wifi ou par le GPS. On surveille donc en tache de fond, dans le
  manager, plutot que d'attendre l'ouverture du panneau. Une simple scrutation suffit :
  la verification ne coute qu'un stat, et rien ne presse a la seconde pres.
  """
  if _clock_is_set():
    _recalibrate()
    return None

  def wait_then_fix() -> None:
    deadline = time.monotonic() + wait
    while time.monotonic() < deadline:
      time.sleep(poll)
      if _clock_is_set():
        try:
          _recalibrate()
        except Exception:
          pass
        return

  thread = threading.Thread(target=wait_then_fix, name="dtc_log_time", daemon=True)
  thread.start()
  return thread


def scan_at_boot(timeout: float = SCAN_TIMEOUT) -> None:
  """Lance la lecture dans un sous-processus. Ne leve jamais, ne bloque jamais
  plus de `timeout` : le demarrage d'openpilot ne doit pas dependre de ca."""
  # Les lignes du scan sont datees par une horloge encore fausse : on les recalera des
  # que possible, ici ou au prochain demarrage du manager (un bouton du panneau relance
  # openpilot sans redemarrer le systeme, l'horloge est alors deja a l'heure).
  fix_log_when_clock_is_set()

  _scan(timeout, "demarrage")


def _scan(timeout: float, origine: str = "") -> None:
  """`origine` est reportee dans le journal : une serie de corrections ne se lit qu'en
  sachant a quel moment chaque point a ete pris."""
  if os.path.exists(DISABLE_PATH):
    return
  cmd = [sys.executable, __file__] + ([origine] if origine else [])
  try:
    subprocess.run(cmd, timeout=timeout, capture_output=True, check=False)
  except Exception:
    pass


def scan_on_ignition(timeout: float = IGNITION_SCAN_TIMEOUT) -> None:
  """Lecture a chaque passage du contact, pour le suivi dans le temps.

  Le demarrage du manager est la seule autre fenetre ou le panda est libre, et elle est
  mauvaise deux fois : elle tombe contact coupe, et le panda interne n'a pas toujours fini sa
  propre sequence. Sur le journal reel, 36 % des tentatives echouaient sur « moteur eteint »
  et 21 % sur un panda indisponible, pour 36 % de releves utiles.

  Au passage du contact les calculateurs repondent et le manager peut rendre le panda
  proprement. Le front montant tient lieu de garde a l'arret : on ne met pas le contact d'une
  voiture qui roule.

  A chaque contact et non sur demande : le suivi du melange pauvre a besoin d'une serie dans
  le temps, pas d'une mesure quand on y pense. Ce que ca coute est borne par `timeout` et
  paye une fois par trajet. La correction long terme est la valeur apprise par le
  calculateur : elle est donc lisible des le contact, moteur encore froid ou meme a l'arret -
  ce qui compte sur un hybride rechargeable, ou le moteur thermique peut ne jamais demarrer.
  La correction court terme, elle, ne vaut quelque chose que moteur tournant.
  """
  _scan(timeout, "contact")


def scan_on_ignition_off(timeout: float = IGNITION_SCAN_TIMEOUT) -> None:
  """Lecture au contact coupe : le seul moment ou les corrections valent quelque chose.

  Mesure sur les deux premiers releves pris au contact mis : liquide a 16 et 18 °C pour un air
  admis a 14 et 15, regime et charge nuls, collecteur a la pression atmospherique. Le moteur
  thermique avait eu toute la nuit pour refroidir, et la correction long terme lue - 4,7 % -
  est celle de la cellule de ralenti, pas celle du point ou le defaut se declenche.

  Ici le moteur vient eventuellement de tourner : le liquide est encore chaud et la correction
  est celle qui vient d'etre apprise en charge. La temperature du liquide sert de juge, au-dela
  de 70 °C la ligne est exploitable. En dessous, le trajet s'est fait en electrique et la
  ligne n'apprend rien - ce qui est une information en soi sur un hybride rechargeable.

  Le delai ne coute rien ici, personne n'attend qu'openpilot soit pret. Le risque est a
  l'autre bout : les calculateurs s'endorment vite contact coupe, et le journal dira si la
  fenetre est trop courte - les lignes porteraient alors « moteur eteint ».
  """
  _scan(timeout, "coupure")


def _read(uds) -> dict:
  from opendbc.car.uds import DTC_REPORT_TYPE

  out: dict = {}
  d = uds._uds_request(0x01, subfunction=0x01)  # noqa: SLF001  mode 01 PID 01
  out["mil"] = bool(d[0] & 0x80)
  out["n_emissions_dtc"] = d[0] & 0x7F
  out["monitors"] = d.hex()

  for mode, key in ((0x03, "stored"), (0x07, "pending")):
    d = uds._uds_request(mode)  # noqa: SLF001
    body = d[1:] if len(d) and d[0] <= 16 else d
    out[key] = sorted({dtc_str(body[i], body[i + 1]) for i in range(0, len(body) - 1, 2)
                       if body[i] or body[i + 1]})

  for mask, key in ((0x08, "confirmed"), (0x20, "failed_since_clear")):
    d = uds.read_dtc_information(DTC_REPORT_TYPE.DTC_BY_STATUS_MASK, dtc_status_mask_type=mask)
    body = d[1:]
    out[key] = sorted({dtc_str(body[i], body[i + 1]) for i in range(0, len(body) - 3, 4)
                       if body[i + 3] & mask})

  out["live"] = _live_values(uds)
  out["freeze"] = _freeze_frame(uds)
  return out


def _live_values(uds) -> dict:
  """Valeurs vivantes du moteur, mode 01. Un PID refuse est simplement absent du resultat.

  Les deux corrections de carburant sont la mesure qui separe les causes d'un melange
  pauvre : une correction long terme franchement positive au ralenti qui se reduit en
  charge designe une entree d'air non mesuree, positive surtout en charge elle designe
  l'alimentation en carburant.
  """
  out: dict = {}
  for pid, nom, taille, decode in PIDS:
    try:
      d = uds._uds_request(0x01, subfunction=pid)  # noqa: SLF001
    except Exception:
      continue
    if len(d) >= taille:
      out[nom] = decode(d)
  return out


def _freeze_frame(uds) -> dict:
  """Trame figee, mode 02 : les conditions moteur a l'instant ou le defaut a ete memorise.

  C'est l'artefact le plus utile du diagnostic, et chaque effacement le detruit. Il est donc
  lu a chaque releve, ce qui le fait capturer par la lecture que _run effectue juste avant
  d'effacer, sans rien changer a la facon de demander un effacement.

  Attention au decalage : la reponse du mode 02 commence par le numero de trame, la ou le
  mode 01 donne directement la valeur, car _uds_request ne retire que l'identifiant de
  service et l'echo du PID. D'ou le d[1:].
  """
  out: dict = {}
  try:
    d = uds._uds_request(0x02, subfunction=0x02, data=b"\x00")  # noqa: SLF001
  except Exception:
    return out
  # Pas de trame tant que rien n'est memorise : le code fige vaut alors zero.
  if len(d) < 3 or not (d[1] or d[2]):
    return out

  out["code"] = dtc_str(d[1], d[2])
  for pid, nom, taille, decode in PIDS:
    try:
      d = uds._uds_request(0x02, subfunction=pid, data=b"\x00")  # noqa: SLF001
    except Exception:
      continue
    if len(d) >= 1 + taille:
      out[nom] = decode(d[1:])
  return out


def _read_confirmed(uds) -> list[str]:
  """Codes confirmes seulement : rapide, pour les calculateurs secondaires."""
  from opendbc.car.uds import DTC_REPORT_TYPE

  d = uds.read_dtc_information(DTC_REPORT_TYPE.DTC_BY_STATUS_MASK, dtc_status_mask_type=0x08)
  body = d[1:]
  return sorted({dtc_str(body[i], body[i + 1]) for i in range(0, len(body) - 3, 4) if body[i + 3] & 0x08})


def _restore_scc(p) -> None:
  """Rend la parole au calculateur SCC : session etendue puis UDS 0x28 « active
  emission et reception ». Sans effet s'il parle deja, il repond simplement oui."""
  from opendbc.car.uds import CONTROL_TYPE, MESSAGE_TYPE, SESSION_TYPE, UdsClient

  uds = UdsClient(p, SCC_ECU, bus=BUS, timeout=0.5, tx_timeout=0.5)
  uds.diagnostic_session_control(SESSION_TYPE.EXTENDED_DIAGNOSTIC)
  uds.communication_control(CONTROL_TYPE.ENABLE_RX_ENABLE_TX, MESSAGE_TYPE.NORMAL)


def _read_others(p, ecus: dict) -> dict:
  """{nom: codes confirmes} pour les calculateurs qui repondent, les autres sont ignores."""
  from opendbc.car.uds import UdsClient

  out: dict[str, list[str]] = {}
  for addr, name in ecus.items():
    try:
      other = UdsClient(p, addr, bus=BUS, timeout=0.3, tx_timeout=0.3)
      other.tester_present()
      codes = _read_confirmed(other)
      if codes:
        out[name] = codes
    except Exception:
      continue
  return out


def _clear(p) -> dict:
  """Efface les codes du moteur et des calculateurs chassis. Retourne {nom: True ou erreur}."""
  from opendbc.car.uds import DTC_GROUP_TYPE, UdsClient

  done: dict[str, object] = {}
  for addr, name in CLEAR_ECUS.items():
    try:
      client = UdsClient(p, addr, bus=BUS, timeout=0.5, tx_timeout=0.5)
      client.tester_present()
      client.clear_diagnostic_information(DTC_GROUP_TYPE.ALL)
      done[name] = True
    except Exception as e:
      done[name] = "%s: %s" % (type(e).__name__, e)
  return done


def _run(do_clear: bool) -> dict:
  from opendbc.car.structs import CarParams
  from opendbc.car.uds import UdsClient
  from panda import Panda

  # mono et boot_id : l'horloge n'est pas encore a l'heure ici, voir _with_corrected_time
  status: dict = {"time": time.time(), "mono": time.monotonic(), "boot_id": _boot_id(),
                  "ok": False, "cleared": None, "error": None, "scc_restored": None}
  p = None
  try:
    p = Panda()
    p.set_safety_mode(int(CarParams.SafetyModel.elm327))
    p.set_obd(True)

    # Au plus tot : le SCC est encore baillonne par le cycle precedent d'openpilot.
    try:
      _restore_scc(p)
      status["scc_restored"] = True
    except Exception as e:
      status["scc_restored"] = "%s: %s" % (type(e).__name__, e)

    uds = UdsClient(p, ENGINE_ECU, bus=BUS, timeout=0.5, tx_timeout=0.5)
    try:
      uds.tester_present()
      status.update(_read(uds))
    except Exception:
      # Le comma se reveille aussi sur une simple activite du bus, a l'approche des cles :
      # le contact est coupe, les calculateurs dorment. Ce n'est pas une panne de lecture.
      status["engine_off"] = True
      raise
    status["ok"] = True

    if do_clear:
      before = sorted(set(status["stored"]) | set(status["confirmed"]) | set(status["pending"]))
      before_chassis = _read_others(p, CHASSIS_ECUS)
      # La trame figee est detruite par l'effacement, et la relecture qui suit la remplacerait
      # par une trame vide : on la met de cote avant, c'est tout l'interet de la lire.
      before_freeze = status.get("freeze") or {}
      status["clear_results"] = _clear(p)
      time.sleep(1.0)
      status.update(_read(uds))
      status["cleared"] = before
      status["cleared_chassis"] = before_chassis
      status["cleared_freeze"] = before_freeze
      _log("efface %s | moteur : %s | chassis : %s" % (
        status["clear_results"], before or "rien", before_chassis or "rien"))
      if before_freeze:
        _log("trame figee effacee, conservee ici : %s" % _fmt(before_freeze))

    status["other_ecus"] = _read_others(p, OTHER_ECUS)

    try:
      cluster = UdsClient(p, CLUSTER_ECU, bus=BUS, timeout=0.3, tx_timeout=0.3)
      cluster.tester_present()
      d = cluster.read_data_by_identifier(ODO_DID)
      if len(d) >= ODO_OFFSET + 3:
        status["odometer_km"] = int.from_bytes(d[ODO_OFFSET:ODO_OFFSET + 3], "big")
    except Exception:
      pass
  except Exception as e:
    status["error"] = "%s: %s" % (type(e).__name__, e)
  finally:
    if p is not None:
      try:
        p.set_obd(False)
        p.set_safety_mode(int(CarParams.SafetyModel.silent))
      except Exception:
        pass
      # Close explicitly and let the bus settle before pandad takes over: handing the
      # panda over too quickly has already produced SPI NACKs on this device.
      try:
        p.close()
      except Exception:
        pass
      time.sleep(HANDOVER_DELAY)
  return status


def _keep_previous_if_engine_off(status: dict) -> dict:
  """Une lecture contact coupe n'apprend rien : elle ne doit pas effacer la precedente.

  Le comma se reveille sur activite du bus, cles approchees, moteur eteint : les
  calculateurs ne repondent pas. On conserve alors le dernier vrai releve, en notant
  la tentative pour que le panneau puisse le dire.
  """
  if status.get("ok") or not status.get("engine_off"):
    return status

  try:
    with open(STATUS_PATH) as f:
      previous = json.load(f)
  except (OSError, ValueError):
    return status

  if not previous.get("ok"):
    return status

  previous["attempt"] = {"time": status["time"], "mono": status["mono"],
                         "boot_id": status["boot_id"], "engine_off": True}
  return previous


def main() -> int:
  origine = sys.argv[1] if len(sys.argv) > 1 else ""
  action = ""
  try:
    with open(REQUEST_PATH) as f:
      action = f.read().strip()
  except OSError:
    pass
  finally:
    try:
      os.unlink(REQUEST_PATH)
    except OSError:
      pass

  status = _keep_previous_if_engine_off(_run(do_clear=(action == "clear")))
  status["origin"] = origine
  try:
    with open(STATUS_PATH, "w") as f:
      json.dump(status, f)
  except OSError:
    pass

  if status.get("attempt", {}).get("engine_off"):
    _log("%smoteur eteint, rien a lire : on garde le releve du %s"
         % ("[%s] " % origine if origine else "",
            time.strftime(TIME_FORMAT, time.localtime(status.get("time", 0)))))
  elif status["ok"]:
    _log("%s%s km | voyant=%s memorises=%s en_attente=%s confirmes=%s echoues_depuis_effacement=%s "
         "scc_reactive=%s autres=%s" % (
           "[%s] " % origine if origine else "",
           status.get("odometer_km", "?"), "ALLUME" if status["mil"] else "eteint",
           status["stored"] or "-", status["pending"] or "-",
           status["confirmed"] or "-", status["failed_since_clear"] or "-",
           status["scc_restored"], status.get("other_ecus") or "-"))
    # Deux lignes a part plutot qu'une ligne a rallonge : les mesures se relisent en colonne
    # d'un releve a l'autre, et une trame figee doit sauter aux yeux quand elle existe.
    if status.get("live"):
      _log("  moteur : %s" % _fmt(status["live"]))
    if status.get("freeze"):
      _log("  trame figee : %s" % _fmt(status["freeze"]))
  else:
    _log("%slecture impossible : %s" % ("[%s] " % origine if origine else "", status["error"]))
  return 0


if __name__ == "__main__":
  sys.exit(main())
