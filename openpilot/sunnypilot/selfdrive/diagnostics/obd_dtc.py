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
import subprocess
import sys
import time

STATUS_PATH = "/data/dtc_status.json"
REQUEST_PATH = "/data/dtc_request"
LOG_PATH = "/data/dtc_log.txt"
DISABLE_PATH = "/data/dtc_disable"

ENGINE_ECU = 0x7E0
SCC_ECU = 0x7D0
ABS_ECU = 0x7D1
BUS = 1
# Calculateurs qui repondent en diagnostic sur cette voiture (scan sur bus 1, OBD actif).
OTHER_ECUS = {0x7E1: "trans", SCC_ECU: "scc", ABS_ECU: "abs", 0x7D4: "eps",
              0x7C6: "cluster", 0x7B7: "corner radar", 0x7B3: "0x7B3"}
# Les codes en C du SCC et de l'ABS sont provoques par openpilot lui-meme, qui baillonne
# le SCC : ces deux modules journalisent la perte de communication. L'effacement les vise
# donc avec le moteur. Les autres calculateurs sont affiches mais jamais effaces.
CHASSIS_ECUS = {SCC_ECU: "scc", ABS_ECU: "abs"}
CLEAR_ECUS = {ENGINE_ECU: "moteur", **CHASSIS_ECUS}
# Odometre : combine (0x7C6), DID 0xB002, octets 6-8 en gros-boutiste, en km.
# Verifie contre le tableau de bord : 131404.
CLUSTER_ECU = 0x7C6
ODO_DID = 0xB002
ODO_OFFSET = 6
SCAN_TIMEOUT = 30.
HANDOVER_DELAY = 1.0
LETTERS = "PCBU"


def dtc_str(b0: int, b1: int) -> str:
  return "%s%d%X%02X" % (LETTERS[(b0 >> 6) & 3], (b0 >> 4) & 3, b0 & 0xF, b1)


def _log(msg: str) -> None:
  """Horodate aussi en secondes depuis le demarrage : au boot l'horloge murale est
  encore a la date par defaut d'AGNOS, le temps monotone permet de s'y retrouver."""
  try:
    with open(LOG_PATH, "a") as f:
      f.write("%s (t+%ds) %s\n" % (time.strftime("%Y-%m-%d %H:%M:%S"), time.monotonic(), msg))
  except OSError:
    pass


# ---------------------------------------------------------------- cote interface

def read_status() -> dict:
  """Dernier releve. N'importe rien de lourd : appele depuis le process UI."""
  try:
    with open(STATUS_PATH) as f:
      return _with_corrected_time(json.load(f))
  except (OSError, ValueError):
    return {}


def _boot_id() -> str:
  try:
    with open("/proc/sys/kernel/random/boot_id") as f:
      return f.read().strip()
  except OSError:
    return ""


def _with_corrected_time(status: dict) -> dict:
  """Recale l'horodatage du releve.

  Au demarrage, le comma n'a ni GPS ni wifi : son horloge est encore a la date par
  defaut d'AGNOS, et le releve est donc horodate n'importe quand. Le temps monotone,
  lui, est juste. Tant qu'on est dans le meme demarrage, on retrouve l'heure reelle
  du releve des que l'horloge est recalee. On reecrit alors le fichier, pour que les
  demarrages suivants lisent la bonne heure.
  """
  mono, boot = status.get("mono"), status.get("boot_id")
  if mono is None or not boot or boot != _boot_id():
    return status

  corrected = time.time() - (time.monotonic() - mono)
  if abs(corrected - status.get("time", 0)) > 60:
    status["time"] = corrected
    try:
      with open(STATUS_PATH, "w") as f:
        json.dump(status, f)
    except OSError:
      pass
  return status


def request(action: str) -> None:
  """'read' ou 'clear', traite au prochain demarrage du manager."""
  try:
    with open(REQUEST_PATH, "w") as f:
      f.write(action)
  except OSError:
    pass


# ---------------------------------------------------------------- cote demarrage

def scan_at_boot(timeout: float = SCAN_TIMEOUT) -> None:
  """Lance la lecture dans un sous-processus. Ne leve jamais, ne bloque jamais
  plus de `timeout` : le demarrage d'openpilot ne doit pas dependre de ca."""
  if os.path.exists(DISABLE_PATH):
    return
  try:
    subprocess.run([sys.executable, __file__], timeout=timeout, capture_output=True, check=False)
  except Exception:
    pass


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
  """Efface les codes du moteur, du SCC et de l'ABS. Retourne {nom: True ou erreur}."""
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
    uds.tester_present()
    status.update(_read(uds))
    status["ok"] = True

    if do_clear:
      before = sorted(set(status["stored"]) | set(status["confirmed"]) | set(status["pending"]))
      before_chassis = _read_others(p, CHASSIS_ECUS)
      status["clear_results"] = _clear(p)
      time.sleep(1.0)
      status.update(_read(uds))
      status["cleared"] = before
      status["cleared_chassis"] = before_chassis
      _log("efface %s | moteur : %s | chassis : %s" % (
        status["clear_results"], before or "rien", before_chassis or "rien"))

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


def main() -> int:
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

  status = _run(do_clear=(action == "clear"))
  try:
    with open(STATUS_PATH, "w") as f:
      json.dump(status, f)
  except OSError:
    pass

  if status["ok"]:
    _log("%s km | voyant=%s memorises=%s en_attente=%s confirmes=%s echoues_depuis_effacement=%s "
         "scc_reactive=%s autres=%s" % (
           status.get("odometer_km", "?"), "ALLUME" if status["mil"] else "eteint",
           status["stored"] or "-", status["pending"] or "-",
           status["confirmed"] or "-", status["failed_since_clear"] or "-",
           status["scc_restored"], status.get("other_ecus") or "-"))
  else:
    _log("lecture impossible : %s" % status["error"])
  return 0


if __name__ == "__main__":
  sys.exit(main())
