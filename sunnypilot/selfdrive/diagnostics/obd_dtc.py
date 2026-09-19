#!/usr/bin/env python3
"""
Lecture et effacement des codes defaut moteur depuis le comma.

Chemin : prise OBD -> comma power -> panda, bus 1 avec le multiplexage OBD actif.
Mesure sur la voiture : le bus 0 (C-CAN du harness camera) ne repond pas en
diagnostic ; 0x7E0 et 0x7E1 repondent sur le bus 1 une fois set_obd(True) pose.

Le panda n'accepte qu'un seul maitre, et interroger un calculateur impose de le
basculer en mode ELM327, ce qui lui retire sa capacite a commander la voiture.
Tout se fait donc au demarrage du manager, avant que pandad ne prenne le panda :
le contact vient d'etre mis, la voiture est a l'arret, le panda est libre.

L'interface (panneau Diagnostics) ne fait que lire le resultat, et au besoin
poser un drapeau puis relancer openpilot.

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
BUS = 1
# Calculateurs qui repondent en diagnostic sur cette voiture (scan sur bus 1, OBD actif).
# Les codes en C du SCC / ABS / EPS sont provoques par openpilot lui-meme, qui desactive
# le calculateur SCC : les autres modules journalisent une perte de communication. On les
# affiche, mais on n'efface que le moteur.
OTHER_ECUS = {0x7E1: "trans", 0x7D0: "scc", 0x7D1: "abs", 0x7D4: "eps",
              0x7C6: "cluster", 0x7B7: "corner radar", 0x7B3: "0x7B3"}
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
  try:
    with open(LOG_PATH, "a") as f:
      f.write("%s %s\n" % (time.strftime("%Y-%m-%d %H:%M:%S"), msg))
  except OSError:
    pass


# ---------------------------------------------------------------- cote interface

def read_status() -> dict:
  """Dernier releve. N'importe rien de lourd : appele depuis le process UI."""
  try:
    with open(STATUS_PATH) as f:
      return json.load(f)
  except (OSError, ValueError):
    return {}


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


def _run(do_clear: bool) -> dict:
  from opendbc.car.structs import CarParams
  from opendbc.car.uds import DTC_GROUP_TYPE, UdsClient
  from panda import Panda

  status: dict = {"time": time.time(), "ok": False, "cleared": None, "error": None}
  p = None
  try:
    p = Panda()
    p.set_safety_mode(int(CarParams.SafetyModel.elm327))
    p.set_obd(True)
    uds = UdsClient(p, ENGINE_ECU, bus=BUS, timeout=0.5, tx_timeout=0.5)
    uds.tester_present()
    status.update(_read(uds))
    status["ok"] = True

    if do_clear:
      before = sorted(set(status["stored"]) | set(status["confirmed"]) | set(status["pending"]))
      uds.clear_diagnostic_information(DTC_GROUP_TYPE.ALL)
      time.sleep(1.0)
      status.update(_read(uds))
      status["cleared"] = before
      _log("efface (moteur) : %s" % (before or "rien"))

    others: dict[str, list[str]] = {}
    for addr, name in OTHER_ECUS.items():
      try:
        other = UdsClient(p, addr, bus=BUS, timeout=0.3, tx_timeout=0.3)
        other.tester_present()
        codes = _read_confirmed(other)
        if codes:
          others[name] = codes
      except Exception:
        continue
    status["other_ecus"] = others

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
    _log("%s km | voyant=%s memorises=%s en_attente=%s confirmes=%s echoues_depuis_effacement=%s" % (
      status.get("odometer_km", "?"), "ALLUME" if status["mil"] else "eteint",
      status["stored"] or "-", status["pending"] or "-",
      status["confirmed"] or "-", status["failed_since_clear"] or "-"))
  else:
    _log("lecture impossible : %s" % status["error"])
  return 0


if __name__ == "__main__":
  sys.exit(main())
