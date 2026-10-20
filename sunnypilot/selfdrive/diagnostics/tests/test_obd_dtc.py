"""Trames de diagnostic envoyees par obd_dtc, verifiees sans la voiture."""
import json
import sys
import time
import types

import pytest

from opendbc.car.uds import SERVICE_TYPE

from openpilot.sunnypilot.selfdrive.diagnostics import obd_dtc

SCC, ABS, EPS, ENGINE = obd_dtc.SCC_ECU, obd_dtc.ABS_ECU, obd_dtc.EPS_ECU, obd_dtc.ENGINE_ECU


class FakePanda:
  """Panda minimal pour UdsClient : repond en ISO-TP trame simple, ou se tait."""

  def __init__(self, answering, dtcs=None, pids=None, freeze=None):
    self.answering = set(answering)
    self.dtcs = dtcs or {}  # {adresse: [(octet0, octet1)]}, un code au plus (trame simple)
    self.pids = pids or {}      # mode 01 : {pid: octets de donnees}
    self.freeze = dict(freeze or {})  # mode 02 : idem, 0x02 portant le code qui a fige
    self.sent: list[tuple[int, bytes, int]] = []
    self._queue: list[tuple[int, bytes, int]] = []

  def can_send(self, addr, dat, bus, timeout=0):
    dat = bytes(dat)
    self.sent.append((addr, dat, bus))
    if addr not in self.answering:
      return
    resp = self._reply(addr, dat)
    if resp is not None:
      self._queue.append((addr + 8, bytes([len(resp)]) + resp.ljust(7, b"\x00"), bus))

  def can_recv(self):
    out, self._queue = self._queue, []
    return out

  def _reply(self, addr: int, dat: bytes) -> bytes | None:
    sid = dat[1]
    if sid in (SERVICE_TYPE.TESTER_PRESENT, SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL,
               SERVICE_TYPE.COMMUNICATION_CONTROL):
      return bytes([sid + 0x40, dat[2]])
    if sid == 0x01:  # OBD mode 01
      if dat[2] in self.pids:
        return bytes([0x41, dat[2]]) + self.pids[dat[2]]
      return bytes([0x41, dat[2], 0x00, 0x07, 0xE5, 0x00])  # voyant eteint, aucun code
    if sid == 0x02:  # OBD mode 02 : la reponse porte le numero de trame avant la valeur
      if dat[2] in self.freeze:
        return bytes([0x42, dat[2], 0x00]) + self.freeze[dat[2]]
      return None
    if sid in (0x03, 0x07):  # OBD modes 03 / 07 : aucun code
      return bytes([sid + 0x40, 0x00])
    if sid == SERVICE_TYPE.CLEAR_DIAGNOSTIC_INFORMATION:
      self.freeze = {}   # un vrai calculateur perd sa trame figee en effacant
      return bytes([sid + 0x40])
    if sid == SERVICE_TYPE.READ_DTC_INFORMATION:
      records = b"".join(bytes([b0, b1, 0x00, 0x08]) for b0, b1 in self.dtcs.get(addr, []))
      return bytes([sid + 0x40, dat[2], 0xFF]) + records
    return None

  def requests_to(self, addr: int) -> list[bytes]:
    return [dat for a, dat, _ in self.sent if a == addr]


class TestRestoreScc:
  def test_sends_extended_session_then_enable_rx_tx(self):
    p = FakePanda([SCC])
    obd_dtc._restore_scc(p)

    # 0x10 0x03 (session etendue) puis 0x28 0x00 0x01 (active emission et reception, normal)
    assert [dat[:4] for dat in p.requests_to(SCC)] == [b"\x02\x10\x03\x00", b"\x03\x28\x00\x01"]
    assert {bus for _, _, bus in p.sent} == {obd_dtc.BUS}

  def test_never_sends_the_disable_variant(self):
    p = FakePanda([SCC])
    obd_dtc._restore_scc(p)

    # 0x28 0x03 couperait le SCC au lieu de le rendre a la voiture
    assert not any(dat[1:3] == b"\x28\x03" for dat in p.requests_to(SCC))

  def test_raises_when_the_scc_stays_silent(self):
    p = FakePanda([])
    with pytest.raises(Exception):
      obd_dtc._restore_scc(p)


class TestClear:
  def test_clears_engine_scc_abs_and_eps(self):
    p = FakePanda([ENGINE, SCC, ABS, EPS])
    assert obd_dtc._clear(p) == {"moteur": True, "scc": True, "abs": True, "eps": True}

    for addr in (ENGINE, SCC, ABS, EPS):
      assert b"\x04\x14\xff\xff\xff" in [dat[:5] for dat in p.requests_to(addr)]

  def test_a_silent_ecu_is_reported_not_raised(self):
    p = FakePanda([ENGINE])
    results = obd_dtc._clear(p)

    assert results["moteur"] is True
    assert all(results[name] is not True for name in obd_dtc.CHASSIS_ECUS.values())

  def test_leaves_the_other_ecus_alone(self):
    p = FakePanda([ENGINE, SCC, ABS, EPS])
    obd_dtc._clear(p)

    spared = set(obd_dtc.OTHER_ECUS) - set(obd_dtc.CLEAR_ECUS)
    assert spared and all(not p.requests_to(addr) for addr in spared)


class TestLiveValues:
  """Mode 01 : les corrections de carburant sont la mesure qui separe les causes d'un
  melange pauvre, le reste donne les conditions du releve."""

  def lire(self, pids):
    from opendbc.car.uds import UdsClient
    p = FakePanda([ENGINE], pids=pids)
    return obd_dtc._live_values(UdsClient(p, ENGINE, bus=obd_dtc.BUS, timeout=0.1, tx_timeout=0.1))

  def test_the_fuel_trims_are_decoded_around_their_zero(self):
    # 0x80 est le zero des corrections, 0x90 vaut +12,5 % (SAE J1979)
    vals = self.lire({0x06: b"\x80", 0x07: b"\x90"})

    assert vals["trim_court_pct"] == 0.0
    assert vals["trim_long_pct"] == 12.5

  def test_a_negative_trim_comes_out_negative(self):
    assert self.lire({0x07: b"\x70"})["trim_long_pct"] == -12.5

  def test_two_byte_pids_are_read_as_two_bytes(self):
    vals = self.lire({0x0C: b"\x0f\xa0", 0x10: b"\x09\xc4"})

    assert vals["regime_tr"] == 1000       # ((0x0f << 8) | 0xa0) / 4
    assert vals["debit_air_gs"] == 25.0    # 2500 / 100

  def test_temperatures_carry_their_offset(self):
    vals = self.lire({0x05: b"\x5a", 0x0F: b"\x28"})

    assert vals["liquide_c"] == 50         # 0x5a - 40
    assert vals["air_admis_c"] == 0

  def test_a_refused_pid_is_simply_absent(self):
    vals = self.lire({0x06: b"\x80"})

    assert "trim_court_pct" in vals
    # le faux calculateur repond a tout en mode 01, mais sur deux octets seulement pour
    # les PID qui en demandent deux : ceux-la sont ecartes plutot que mal decodes
    assert set(vals) <= {nom for _, nom, _, _ in obd_dtc.PIDS}


class TestFreezeFrame:
  """Mode 02 : les conditions moteur a l'instant du defaut, detruites par chaque effacement."""

  def lire(self, freeze):
    from opendbc.car.uds import UdsClient
    p = FakePanda([ENGINE], freeze=freeze)
    return obd_dtc._freeze_frame(UdsClient(p, ENGINE, bus=obd_dtc.BUS, timeout=0.1, tx_timeout=0.1))

  def test_it_says_which_code_froze_the_frame(self):
    frame = self.lire({0x02: b"\x01\x71"})

    assert frame["code"] == "P0171"

  def test_the_frame_number_is_not_mistaken_for_the_value(self):
    # La reponse du mode 02 est 42 <PID> <numero de trame> <valeur> : sans le decalage,
    # on lirait le numero de trame, soit zero, au lieu de la mesure.
    frame = self.lire({0x02: b"\x01\x71", 0x07: b"\x90", 0x0C: b"\x0f\xa0"})

    assert frame["trim_long_pct"] == 12.5
    assert frame["regime_tr"] == 1000

  def test_no_frame_when_nothing_is_stored(self):
    # Le code fige vaut zero tant qu'aucun defaut n'est memorise.
    assert self.lire({0x02: b"\x00\x00", 0x07: b"\x90"}) == {}

  def test_an_ecu_without_mode_02_is_not_an_error(self):
    assert self.lire({}) == {}


class TestIgnitionScan:
  """La lecture au contact est le seul moment ou les calculateurs repondent : sur le journal
  reel, 36 % des tentatives echouaient sur « moteur eteint » et 21 % sur un panda
  indisponible, le panda interne n'ayant pas toujours fini sa sequence au demarrage."""

  @pytest.fixture(autouse=True)
  def _paths(self, monkeypatch, tmp_path):
    monkeypatch.setattr(obd_dtc, "REQUEST_PATH", str(tmp_path / "dtc_request"))
    monkeypatch.setattr(obd_dtc, "DISABLE_PATH", str(tmp_path / "dtc_disable"))
    self.lances = []
    monkeypatch.setattr(obd_dtc.subprocess, "run",
                        lambda *a, **k: self.lances.append(k.get("timeout")))

  def test_every_ignition_is_scanned(self):
    # le suivi du melange pauvre a besoin d'une serie dans le temps, pas d'une mesure
    # quand on y pense : aucune demande n'est requise
    assert not obd_dtc.pending_request()
    obd_dtc.scan_on_ignition()

    assert self.lances == [obd_dtc.IGNITION_SCAN_TIMEOUT]

  def test_the_ignition_budget_is_shorter_than_the_boot_one(self):
    # il borne le temps pendant lequel openpilot n'est pas pret, paye a chaque trajet
    assert obd_dtc.IGNITION_SCAN_TIMEOUT < obd_dtc.SCAN_TIMEOUT

  def test_a_pending_clear_is_honoured_by_the_same_scan(self):
    obd_dtc.request("clear")
    assert obd_dtc.pending_request()
    obd_dtc.scan_on_ignition()

    # un seul scan sert les deux : main() lit la demande et efface si elle le dit
    assert self.lances == [obd_dtc.IGNITION_SCAN_TIMEOUT]

  def test_the_disable_file_stops_it(self):
    open(obd_dtc.DISABLE_PATH, "w").close()
    obd_dtc.scan_on_ignition()

    assert self.lances == []

  def test_the_boot_scan_keeps_its_own_budget(self, monkeypatch):
    monkeypatch.setattr(obd_dtc, "fix_log_when_clock_is_set", lambda *a, **k: None)
    obd_dtc.scan_at_boot()

    assert self.lances == [obd_dtc.SCAN_TIMEOUT]


class FakePandaDevice(FakePanda):
  """Ajoute ce que _run appelle sur le panda lui-meme."""

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.calls: list[str] = []

  def set_safety_mode(self, mode):
    self.calls.append("safety=%d" % mode)

  def set_obd(self, enabled):
    self.calls.append("obd=%s" % enabled)

  def close(self):
    self.calls.append("close")


@pytest.fixture
def fake_panda(monkeypatch, tmp_path):
  device = FakePandaDevice(set(obd_dtc.OTHER_ECUS) | {obd_dtc.ENGINE_ECU},
                           dtcs={obd_dtc.SCC_ECU: [(0x56, 0x38)]},
                           pids={0x07: b"\x90"},
                           freeze={0x02: b"\x01\x71", 0x07: b"\x90"})
  monkeypatch.setitem(sys.modules, "panda", types.SimpleNamespace(Panda=lambda: device))
  monkeypatch.setattr(obd_dtc.time, "sleep", lambda _: None)
  monkeypatch.setattr(obd_dtc, "LOG_PATH", str(tmp_path / "dtc_log.txt"))
  return device


class TestRun:
  def test_restores_the_scc_before_reading(self, fake_panda):
    status = obd_dtc._run(do_clear=False)

    assert status["ok"] and status["scc_restored"] is True
    assert status["other_ecus"] == {"scc": ["C1638"]}
    # la reactivation part avant toute interrogation du moteur
    order = [addr for addr, _, _ in fake_panda.sent]
    assert order.index(obd_dtc.SCC_ECU) < order.index(obd_dtc.ENGINE_ECU)

  def test_read_only_run_clears_nothing(self, fake_panda):
    obd_dtc._run(do_clear=False)

    assert not any(dat[:2] == b"\x04\x14" for _, dat, _ in fake_panda.sent)

  def test_clear_run_erases_every_target_and_records_what_was_erased(self, fake_panda):
    status = obd_dtc._run(do_clear=True)

    assert status["clear_results"] == {"moteur": True, "scc": True, "abs": True, "eps": True}
    assert status["cleared_chassis"] == {"scc": ["C1638"]}

  def test_the_frozen_frame_survives_the_clear(self, fake_panda):
    """Tout l'interet de la lire : l'effacement la detruit, et la relecture qui suit la
    remplacerait par une trame vide."""
    status = obd_dtc._run(do_clear=True)

    assert status["cleared_freeze"]["code"] == "P0171"
    assert status["cleared_freeze"]["trim_long_pct"] == 12.5
    assert not status["freeze"]        # le calculateur l'a bien perdue

  def test_a_read_without_clearing_keeps_the_frame_in_place(self, fake_panda):
    status = obd_dtc._run(do_clear=False)

    assert status["freeze"]["code"] == "P0171"
    assert status["live"]["trim_long_pct"] == 12.5

  def test_hands_the_panda_back(self, fake_panda):
    obd_dtc._run(do_clear=False)

    assert fake_panda.calls[-3:] == ["obd=False", "safety=0", "close"]  # 0 = silent


class TestStatusTime:
  """Au demarrage l'horloge du comma est encore a la date par defaut d'AGNOS."""

  def _write(self, monkeypatch, tmp_path, status):
    path = tmp_path / "dtc_status.json"
    path.write_text(json.dumps(status))
    monkeypatch.setattr(obd_dtc, "STATUS_PATH", str(path))
    monkeypatch.setattr(obd_dtc, "LOG_PATH", str(tmp_path / "dtc_log.txt"))
    monkeypatch.setattr(obd_dtc, "_clock_is_set", lambda: True)
    return path

  def test_same_boot_recovers_the_real_time(self, monkeypatch, tmp_path):
    path = self._write(monkeypatch, tmp_path, {"time": 1_700_000_000.0, "mono": time.monotonic() - 10,
                                               "boot_id": obd_dtc._boot_id(), "ok": True})
    status = obd_dtc.read_status()

    assert abs(status["time"] - (time.time() - 10)) < 2
    # corrige aussi dans le fichier, pour les demarrages suivants
    assert json.loads(path.read_text())["time"] == status["time"]

  def test_a_clock_already_right_is_left_alone(self, monkeypatch, tmp_path):
    stamp = time.time() - 30
    self._write(monkeypatch, tmp_path, {"time": stamp, "mono": time.monotonic() - 30,
                                        "boot_id": obd_dtc._boot_id(), "ok": True})
    assert obd_dtc.read_status()["time"] == stamp

  def test_another_boot_is_left_alone(self, monkeypatch, tmp_path):
    self._write(monkeypatch, tmp_path, {"time": 1_700_000_000.0, "mono": 42.0,
                                        "boot_id": "un-autre-demarrage", "ok": True})
    assert obd_dtc.read_status()["time"] == 1_700_000_000.0

  def test_status_without_monotonic_is_left_alone(self, monkeypatch, tmp_path):
    self._write(monkeypatch, tmp_path, {"time": 1_700_000_000.0, "ok": True})
    assert obd_dtc.read_status()["time"] == 1_700_000_000.0

  def test_nothing_is_corrected_while_the_clock_is_wrong(self, monkeypatch, tmp_path):
    self._write(monkeypatch, tmp_path, {"time": 1_700_000_000.0, "mono": time.monotonic() - 10,
                                        "boot_id": obd_dtc._boot_id(), "ok": True})
    monkeypatch.setattr(obd_dtc, "_clock_is_set", lambda: False)
    assert obd_dtc.read_status()["time"] == 1_700_000_000.0

  def test_a_run_records_what_the_correction_needs(self, fake_panda):
    status = obd_dtc._run(do_clear=False)

    assert status["boot_id"] == obd_dtc._boot_id()
    assert abs(status["mono"] - time.monotonic()) < 5


class TestLogTimes:
  """Les lignes ecrites au demarrage portent une fausse heure murale, on les recale."""

  @pytest.fixture(autouse=True)
  def _log_file(self, monkeypatch, tmp_path):
    self.path = tmp_path / "dtc_log.txt"
    monkeypatch.setattr(obd_dtc, "LOG_PATH", str(self.path))
    monkeypatch.setattr(obd_dtc, "_clock_is_set", lambda: True)

  def _line(self, stamp: str, mono: float, boot: str, msg: str = "voyant=eteint") -> str:
    return "%s (t+%ds %s) %s\n" % (stamp, mono, boot, msg)

  def test_log_carries_uptime_and_boot(self):
    obd_dtc._log("coucou")

    assert obd_dtc.LOG_LINE.match(self.path.read_text().rstrip("\n"))

  def test_current_boot_line_is_recalibrated(self):
    mono = time.monotonic() - 20
    self.path.write_text(self._line("2026-03-24 14:46:14", mono, obd_dtc._boot_id()[:8]))
    obd_dtc._fix_log_times()

    m = obd_dtc.LOG_LINE.match(self.path.read_text().rstrip("\n"))
    written = time.mktime(time.strptime(m.group(1), obd_dtc.TIME_FORMAT))
    assert abs(written - (time.time() - 20)) < 5
    assert m.group(4) == "voyant=eteint"  # le message est conserve

  def test_other_boots_and_legacy_lines_are_untouched(self):
    original = (self._line("2026-03-24 14:46:14", 31, "deadbeef") +
                "2026-09-19 07:32:17 efface : ['P0171']\n")
    self.path.write_text(original)
    obd_dtc._fix_log_times()

    assert self.path.read_text() == original

  def test_correct_lines_are_left_alone(self):
    mono = time.monotonic() - 5
    original = self._line(time.strftime(obd_dtc.TIME_FORMAT, time.localtime(time.time() - 5)),
                          mono, obd_dtc._boot_id()[:8])
    self.path.write_text(original)
    obd_dtc._fix_log_times()

    assert self.path.read_text() == original

  def test_a_manager_start_also_recalibrates(self, monkeypatch):
    """Le bouton relance openpilot sans redemarrer le systeme : meme demarrage, horloge juste."""
    monkeypatch.setattr(obd_dtc.subprocess, "run", lambda *a, **k: None)
    self.path.write_text(self._line("2026-03-24 14:46:14", time.monotonic() - 20, obd_dtc._boot_id()[:8]))
    obd_dtc.scan_at_boot()

    m = obd_dtc.LOG_LINE.match(self.path.read_text().rstrip("\n"))
    assert abs(time.mktime(time.strptime(m.group(1), obd_dtc.TIME_FORMAT)) - (time.time() - 20)) < 5

  def test_nothing_is_touched_while_the_clock_is_wrong(self, monkeypatch):
    monkeypatch.setattr(obd_dtc, "_clock_is_set", lambda: False)
    original = self._line("2026-03-24 14:46:14", time.monotonic() - 20, obd_dtc._boot_id()[:8])
    self.path.write_text(original)
    obd_dtc._fix_log_times()

    assert self.path.read_text() == original


class TestEngineOff:
  """Reveil sur activite du bus, cles approchees : les calculateurs dorment."""

  @pytest.fixture(autouse=True)
  def _paths(self, monkeypatch, tmp_path):
    self.status_path = tmp_path / "dtc_status.json"
    monkeypatch.setattr(obd_dtc, "STATUS_PATH", str(self.status_path))
    monkeypatch.setattr(obd_dtc, "LOG_PATH", str(tmp_path / "dtc_log.txt"))
    monkeypatch.setattr(obd_dtc, "REQUEST_PATH", str(tmp_path / "dtc_request"))

  def test_a_run_without_answer_is_flagged_engine_off(self, monkeypatch, tmp_path):
    device = FakePandaDevice([])  # personne ne repond
    monkeypatch.setitem(sys.modules, "panda", types.SimpleNamespace(Panda=lambda: device))
    monkeypatch.setattr(obd_dtc.time, "sleep", lambda _: None)
    status = obd_dtc._run(do_clear=False)

    assert status["engine_off"] is True and status["ok"] is False

  def test_the_previous_reading_is_kept(self, monkeypatch):
    good = {"time": 1_774_000_000.0, "mono": 12.0, "boot_id": "avant", "ok": True,
            "mil": False, "stored": [], "pending": [], "confirmed": [], "failed_since_clear": [],
            "scc_restored": True, "other_ecus": {"scc": ["C1638"]}}
    self.status_path.write_text(json.dumps(good))
    monkeypatch.setattr(obd_dtc, "_run", lambda do_clear: {"time": time.time(), "mono": time.monotonic(),
                                                           "boot_id": obd_dtc._boot_id(), "ok": False,
                                                           "engine_off": True, "error": "timeout"})
    obd_dtc.main()

    kept = json.loads(self.status_path.read_text())
    assert kept["ok"] is True and kept["other_ecus"] == {"scc": ["C1638"]}
    assert kept["time"] == good["time"]
    assert kept["attempt"]["engine_off"] is True   # le panneau peut le signaler

  def test_a_real_failure_still_replaces_the_reading(self, monkeypatch):
    self.status_path.write_text(json.dumps({"time": 1.0, "ok": True, "mil": False, "stored": [],
                                            "pending": [], "confirmed": [], "failed_since_clear": [],
                                            "scc_restored": True}))
    monkeypatch.setattr(obd_dtc, "_run", lambda do_clear: {"time": time.time(), "mono": time.monotonic(),
                                                           "boot_id": obd_dtc._boot_id(), "ok": False,
                                                           "error": "PandaError: pas de panda"})
    obd_dtc.main()

    assert json.loads(self.status_path.read_text())["ok"] is False

  def test_nothing_to_keep_leaves_the_failed_reading(self, monkeypatch):
    monkeypatch.setattr(obd_dtc, "_run", lambda do_clear: {"time": time.time(), "mono": time.monotonic(),
                                                           "boot_id": obd_dtc._boot_id(), "ok": False,
                                                           "engine_off": True, "error": "timeout"})
    obd_dtc.main()

    assert json.loads(self.status_path.read_text())["engine_off"] is True


class TestAutomaticRecalibration:
  """Le recalage ne doit demander aucune action : il attend la mise a l'heure."""

  @pytest.fixture(autouse=True)
  def _paths(self, monkeypatch, tmp_path):
    self.log_path = tmp_path / "dtc_log.txt"
    monkeypatch.setattr(obd_dtc, "LOG_PATH", str(self.log_path))
    monkeypatch.setattr(obd_dtc, "STATUS_PATH", str(tmp_path / "dtc_status.json"))
    self.stale = "2026-03-24 14:46:14 (t+%ds %s) voyant=eteint\n" % (time.monotonic() - 20, obd_dtc._boot_id()[:8])
    self.log_path.write_text(self.stale)

  def _stamp(self) -> float:
    m = obd_dtc.LOG_LINE.match(self.log_path.read_text().rstrip("\n"))
    return time.mktime(time.strptime(m.group(1), obd_dtc.TIME_FORMAT))

  def test_clock_already_set_recalibrates_right_away(self, monkeypatch):
    monkeypatch.setattr(obd_dtc, "_clock_is_set", lambda: True)

    assert obd_dtc.fix_log_when_clock_is_set() is None  # rien a attendre
    assert abs(self._stamp() - (time.time() - 20)) < 5

  def test_it_waits_for_the_clock_on_its_own(self, monkeypatch):
    ready = []
    monkeypatch.setattr(obd_dtc, "_clock_is_set", lambda: bool(ready))

    thread = obd_dtc.fix_log_when_clock_is_set(poll=0.01, wait=10)
    assert thread is not None and self.log_path.read_text() == self.stale  # rien tant que l'heure est fausse

    ready.append(True)
    thread.join(timeout=5)
    assert not thread.is_alive()
    assert abs(self._stamp() - (time.time() - 20)) < 5

  def test_it_gives_up_instead_of_watching_forever(self, monkeypatch):
    monkeypatch.setattr(obd_dtc, "_clock_is_set", lambda: False)

    thread = obd_dtc.fix_log_when_clock_is_set(poll=0.01, wait=0.05)
    thread.join(timeout=5)
    assert not thread.is_alive() and self.log_path.read_text() == self.stale


class TestLogSize:
  """Le journal vit sur /data : il doit rester borne tout seul."""

  @pytest.fixture(autouse=True)
  def _paths(self, monkeypatch, tmp_path):
    self.log_path = tmp_path / "dtc_log.txt"
    monkeypatch.setattr(obd_dtc, "LOG_PATH", str(self.log_path))
    monkeypatch.setattr(obd_dtc, "LOG_MAX_BYTES", 2000)
    monkeypatch.setattr(obd_dtc, "LOG_KEEP_LINES", 5)

  def test_a_small_log_is_left_alone(self):
    for i in range(3):
      obd_dtc._log("ligne %d" % i)

    assert len(self.log_path.read_text().splitlines()) == 3

  def test_an_oversized_log_keeps_only_the_newest_lines(self):
    for i in range(200):
      obd_dtc._log("ligne %d" % i)

    lines = self.log_path.read_text().splitlines()
    assert self.log_path.stat().st_size <= obd_dtc.LOG_MAX_BYTES     # la borne tient
    assert lines[-1].endswith("ligne 199")                           # la derniere est conservee
    assert not any(ln.endswith("ligne 0") for ln in lines)           # les plus vieilles sont parties

  def test_trimmed_lines_stay_readable_for_the_recalibration(self, monkeypatch):
    monkeypatch.setattr(obd_dtc, "_clock_is_set", lambda: True)
    for i in range(200):
      obd_dtc._log("ligne %d" % i)

    assert all(obd_dtc.LOG_LINE.match(ln) for ln in self.log_path.read_text().splitlines())


class TestReadOthers:
  def test_returns_codes_of_answering_ecus(self):
    p = FakePanda([SCC], dtcs={SCC: [(0x56, 0x38)]})  # C1638
    assert obd_dtc._read_others(p, obd_dtc.CHASSIS_ECUS) == {"scc": ["C1638"]}

  def test_silent_ecus_are_skipped(self):
    assert obd_dtc._read_others(FakePanda([]), obd_dtc.CHASSIS_ECUS) == {}
