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

  def __init__(self, answering, dtcs=None):
    self.answering = set(answering)
    self.dtcs = dtcs or {}  # {adresse: [(octet0, octet1)]}, un code au plus (trame simple)
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
    if sid == 0x01:  # OBD mode 01 : voyant moteur eteint, aucun code
      return bytes([0x41, dat[2], 0x00, 0x07, 0xE5, 0x00])
    if sid in (0x03, 0x07):  # OBD modes 03 / 07 : aucun code
      return bytes([sid + 0x40, 0x00])
    if sid == SERVICE_TYPE.CLEAR_DIAGNOSTIC_INFORMATION:
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


class FakePandaDevice(FakePanda):
  """Ajoute ce que _run appelle sur le panda lui-meme."""

  def __init__(self, answering, dtcs=None):
    super().__init__(answering, dtcs)
    self.calls: list[str] = []

  def set_safety_mode(self, mode):
    self.calls.append("safety=%d" % mode)

  def set_obd(self, enabled):
    self.calls.append("obd=%s" % enabled)

  def close(self):
    self.calls.append("close")


@pytest.fixture
def fake_panda(monkeypatch, tmp_path):
  device = FakePandaDevice(set(obd_dtc.OTHER_ECUS) | {obd_dtc.ENGINE_ECU}, dtcs={obd_dtc.SCC_ECU: [(0x56, 0x38)]})
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


class TestReadOthers:
  def test_returns_codes_of_answering_ecus(self):
    p = FakePanda([SCC], dtcs={SCC: [(0x56, 0x38)]})  # C1638
    assert obd_dtc._read_others(p, obd_dtc.CHASSIS_ECUS) == {"scc": ["C1638"]}

  def test_silent_ecus_are_skipped(self):
    assert obd_dtc._read_others(FakePanda([]), obd_dtc.CHASSIS_ECUS) == {}
