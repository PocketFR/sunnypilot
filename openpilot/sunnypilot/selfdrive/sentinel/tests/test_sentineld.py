"""Logique de la sentinelle, verifiee sans voiture ni camera."""
import json
import os
import time

import numpy as np
import pytest

from openpilot.sunnypilot.selfdrive.sentinel import sentineld, state


def frame(width=160, height=120, value=100):
  return np.full((height, width), value, dtype=np.uint8)


@pytest.fixture
def armed(monkeypatch, tmp_path):
  """Arme la sentinelle : un fichier, pas un parametre (voir sentinel/state.py)."""
  path = tmp_path / "sentry_armed"
  monkeypatch.setattr(state, "ARMED_PATH", str(path))
  monkeypatch.setattr(state, "MIN_VOLTAGE_PATH", str(tmp_path / "sentry_min_voltage"))
  state.arm()
  return path


class TestMotionDetector:
  def test_first_frame_never_triggers(self):
    assert sentineld.MotionDetector().update(frame()) is False

  def test_a_still_scene_does_not_trigger(self):
    d = sentineld.MotionDetector()
    d.update(frame())
    assert d.update(frame()) is False

  def test_something_moving_triggers(self):
    d = sentineld.MotionDetector()
    d.update(frame())
    moved = frame()
    moved[20:90, 20:110] = 220  # une silhouette qui traverse
    assert d.update(moved) is True

  def test_a_global_brightness_change_does_not_trigger(self):
    """Un nuage qui passe, un lampadaire qui s'allume : toute l'image bouge d'un bloc."""
    d = sentineld.MotionDetector()
    d.update(frame(value=100))
    assert d.update(frame(value=160)) is False

  def test_sensor_noise_does_not_trigger(self):
    rng = np.random.default_rng(0)
    d = sentineld.MotionDetector()
    base = frame()
    d.update(np.clip(base + rng.integers(-3, 3, base.shape), 0, 255).astype(np.uint8))
    assert d.update(np.clip(base + rng.integers(-3, 3, base.shape), 0, 255).astype(np.uint8)) is False


class TestEventRecorder:
  @pytest.fixture(autouse=True)
  def _root(self, tmp_path):
    self.root = str(tmp_path / "sentry")
    self.rec = sentineld.EventRecorder(self.root, pre_roll=3, post_event_s=5.)

  def _offer_all(self, n=1, payload=b"jpeg"):
    for _ in range(n):
      for cam in sentineld.CAMERAS:
        self.rec.offer(cam, payload)

  def test_nothing_is_written_before_a_trigger(self):
    self._offer_all(5)
    assert not os.path.exists(self.root) or sentineld.event_dirs(self.root) == []

  def test_the_pre_roll_lands_in_the_event(self):
    self._offer_all(5)                      # seules les 3 dernieres sont gardees
    self.rec.trigger("motion", time.monotonic())

    names = sorted(os.listdir(self.rec.path))
    assert len([n for n in names if n.startswith("f_m")]) == 3
    assert "meta.json" in names

  def test_frames_are_written_while_recording(self):
    now = time.monotonic()
    self.rec.trigger("can", now)
    for i in range(3):                      # un tour de boucle = une image par camera, puis tick
      self._offer_all(1)
      self.rec.tick(now + i)

    assert sorted(n for n in os.listdir(self.rec.path) if n.startswith("f_")) == \
           ["f_000.jpg", "f_001.jpg", "f_002.jpg"]

  def test_the_three_cameras_share_the_same_index(self):
    """Le timelapse recolle les trois cameras : leurs numeros doivent correspondre."""
    now = time.monotonic()
    self.rec.trigger("can", now)
    self._offer_all(1)
    self.rec.tick(now)
    self._offer_all(1)

    for cam in sentineld.CAMERAS:
      assert sorted(n for n in os.listdir(self.rec.path) if n.startswith(cam + "_0")) == \
             ["%s_000.jpg" % cam, "%s_001.jpg" % cam]

  def test_a_new_trigger_extends_the_event(self):
    now = time.monotonic()
    self.rec.trigger("motion", now)
    self.rec.trigger("motion", now + 4.)
    self.rec.tick(now + 8.)                 # 4 s apres le dernier declencheur, moins que post_event_s

    assert self.rec.recording

  def test_the_event_closes_after_the_delay(self):
    now = time.monotonic()
    self.rec.trigger("motion", now)
    path = self.rec.path
    self.rec.tick(now + 6.)

    assert not self.rec.recording
    meta = json.loads(open(os.path.join(path, "meta.json")).read())
    assert meta["triggers"] == ["motion"] and "closed" in meta

  def test_triggers_are_recorded_without_duplicates(self):
    now = time.monotonic()
    self.rec.trigger("motion", now)
    self.rec.trigger("can", now)
    self.rec.trigger("motion", now)

    assert self.rec.triggers == ["motion", "can"]


class TestPrune:
  def _event(self, root, name, size, mtime):
    path = os.path.join(root, name)
    os.makedirs(path, exist_ok=True)
    with open(os.path.join(path, "f_000.jpg"), "wb") as f:
      f.write(b"x" * size)
    os.utime(path, (mtime, mtime))
    return path

  def test_the_oldest_events_go_first(self, tmp_path):
    root = str(tmp_path)
    old = self._event(root, "20260101_000000", 1000, 1000)
    recent = self._event(root, "20260102_000000", 1000, 2000)

    removed = sentineld.prune(root, max_bytes=1500)

    assert removed == [old] and os.path.exists(recent)

  def test_nothing_is_removed_under_the_quota(self, tmp_path):
    root = str(tmp_path)
    self._event(root, "20260101_000000", 1000, 1000)

    assert sentineld.prune(root, max_bytes=10_000) == []

  def test_the_event_being_recorded_is_spared(self, tmp_path):
    root = str(tmp_path)
    current = self._event(root, "20260101_000000", 4000, 1000)

    assert sentineld.prune(root, max_bytes=10, keep=current) == []
    assert os.path.exists(current)


class TestRecalibrateEvents:
  """Au reveil l'horloge est a la date de l'image AGNOS : les dossiers sont mal dates."""

  @pytest.fixture(autouse=True)
  def _clock(self, monkeypatch):
    monkeypatch.setattr(sentineld, "clock_is_set", lambda: True)

  def _event(self, root, name, meta):
    path = os.path.join(root, name)
    os.makedirs(path)
    with open(os.path.join(path, "meta.json"), "w") as f:
      json.dump(meta, f)
    return path

  def test_an_event_of_this_boot_is_renamed(self, tmp_path):
    root = str(tmp_path)
    self._event(root, "20260324_144614", {"time": 1_774_359_974.0, "mono": time.monotonic() - 30,
                                          "boot_id": sentineld.boot_id()})
    renamed = sentineld.recalibrate_events(root)

    assert len(renamed) == 1
    target = renamed[0][1]
    assert abs(os.path.getmtime(target) - (time.time() - 30)) < 5
    assert abs(json.loads(open(os.path.join(target, "meta.json")).read())["time"] - (time.time() - 30)) < 5

  def test_another_boot_is_left_alone(self, tmp_path):
    root = str(tmp_path)
    self._event(root, "20260324_144614", {"time": 1_774_359_974.0, "mono": 31.0, "boot_id": "autre"})

    assert sentineld.recalibrate_events(root) == []

  def test_a_well_dated_event_is_left_alone(self, tmp_path):
    root = str(tmp_path)
    self._event(root, "20260920_060000", {"time": time.time() - 10, "mono": time.monotonic() - 10,
                                          "boot_id": sentineld.boot_id()})

    assert sentineld.recalibrate_events(root) == []

  def test_nothing_happens_while_the_clock_is_wrong(self, tmp_path, monkeypatch):
    monkeypatch.setattr(sentineld, "clock_is_set", lambda: False)
    root = str(tmp_path)
    self._event(root, "20260324_144614", {"time": 1.0, "mono": time.monotonic(), "boot_id": sentineld.boot_id()})

    assert sentineld.recalibrate_events(root) == []


class TestVoltageWatchdog:
  def test_a_healthy_battery_never_stops_anything(self):
    w = sentineld.VoltageWatchdog(11800, duration=60.)
    assert w.update(12400, 0.) is False and w.update(12400, 100.) is False

  def test_a_brief_dip_is_tolerated(self):
    """Un demarreur ou une pompe font plonger la tension une poignee de secondes."""
    w = sentineld.VoltageWatchdog(11800, duration=60.)
    assert w.update(11500, 0.) is False
    assert w.update(11500, 30.) is False
    assert w.update(12300, 40.) is False
    assert w.update(11500, 70.) is False   # le compteur est reparti de zero

  def test_a_sustained_drop_stops_the_watch(self):
    w = sentineld.VoltageWatchdog(11800, duration=60.)
    w.update(11500, 0.)
    assert w.update(11500, 61.) is True

  def test_an_unknown_voltage_is_not_a_reason_to_stop(self):
    assert sentineld.VoltageWatchdog(11800).update(0, 0.) is False


class TestSentryStop:
  def _sentry(self, tmp_path):
    return sentineld.Sentry(str(tmp_path / "events"))

  def test_ignition_stops_the_watch(self, tmp_path, armed):
    s = self._sentry(tmp_path)
    assert s.check_stop(ignition=True, voltage_mv=12500, now=0.) == "ignition"

  def test_a_flat_battery_stops_the_watch(self, tmp_path, armed):
    s = self._sentry(tmp_path)
    assert s.check_stop(False, 11000, 0.) is None
    assert s.check_stop(False, 11000, 61.) == "voltage"

  def test_disarming_from_the_screen_stops_the_watch(self, tmp_path, armed):
    s = self._sentry(tmp_path)
    state.disarm()
    assert s.check_stop(False, 12500, 0.) == "disarmed"

  def test_it_keeps_watching_otherwise(self, tmp_path, armed):
    assert self._sentry(tmp_path).check_stop(False, 12500, 0.) is None


class TestSentryFrames:
  def _sentry(self, tmp_path):
    return sentineld.Sentry(str(tmp_path / "events"))

  def _frames(self, still=True):
    moving = frame()
    moving[20:90, 20:110] = 220
    out = {}
    for cam in sentineld.CAMERAS:
      y = frame() if still or cam == "d" else moving
      out[cam] = (y, lambda: b"jpeg")
    return out

  def test_movement_opens_an_event_with_the_three_cameras(self, tmp_path, armed):
    s = self._sentry(tmp_path)
    s.on_frames(self._frames(still=True), 0.)      # premiere image : reference
    s.on_frames(self._frames(still=False), 1.)

    assert s.recorder.recording
    written = os.listdir(s.recorder.path)
    assert {n[0] for n in written if n.endswith(".jpg")} == {"f", "e", "d"}

  def test_the_cabin_camera_triggers_too(self, tmp_path, armed):
    """Elle regarde par les vitres laterales : c'est elle qui voit venir sur les cotes."""
    s = self._sentry(tmp_path)
    moving = frame()
    moving[20:90, 20:110] = 220
    s.on_frames({"d": (frame(), lambda: b"jpeg")}, 0.)
    s.on_frames({"d": (moving, lambda: b"jpeg")}, 1.)

    assert s.recorder.recording and s.recorder.triggers == ["motion"]

  def test_a_can_frame_opens_an_event(self, tmp_path, armed):
    s = self._sentry(tmp_path)
    s.on_can(0., {0})

    assert s.recorder.recording and s.recorder.triggers == ["can"]

  def test_a_full_disk_stops_the_writing_not_the_watch(self, tmp_path, armed):
    s = self._sentry(tmp_path)
    s.on_can(0., {0})
    path = s.recorder.path
    s.disk_full = True
    s.on_frames(self._frames(), 1.)

    assert s.recorder.recording
    assert len([n for n in os.listdir(path) if n.endswith(".jpg")]) == 0


class TestStatus:
  def test_status_says_what_the_panel_shows(self, tmp_path, armed):
    s = sentineld.Sentry(str(tmp_path / "events"))
    s.check_stop(False, 12480, 0.)
    status = s.status()

    assert status["armed"] is True and status["recording"] is False
    assert status["voltage_mv"] == 12480 and status["min_voltage_mv"] == 11800
    assert status["boot_id"] == sentineld.boot_id()


class FakeBuf:
  """Tampon VisionIPC minimal : NV12, avec un stride plus large que l'image."""

  def __init__(self, width=8, height=4, stride=16, y_value=90, as_memoryview=True):
    self.width, self.height, self.stride = width, height, stride
    self.uv_offset = stride * height
    uv_height = ((height // 2) + 15) // 16 * 16
    data = np.full(self.uv_offset + stride * uv_height, 128, dtype=np.uint8)  # U = V = 128 : gris
    y = data[:self.uv_offset].reshape((-1, stride))
    y[:height, :width] = y_value
    y[:height, width:] = 7  # bourrage de ligne : ne doit jamais ressortir
    # memoryview comme sur l'appareil : certaines versions de msgq rendent un tableau
    # numpy, et confondre les deux a deja fait tomber sentineld (buf.data.size)
    self.data = memoryview(data.tobytes()) if as_memoryview else data


class TestFrameExtraction:
  @pytest.mark.parametrize("as_memoryview", [True, False])
  def test_both_buffer_flavours_extract_the_same(self, as_memoryview):
    y = sentineld.extract_y(FakeBuf(y_value=77, as_memoryview=as_memoryview))

    assert y.shape == (4, 8) and (y == 77).all()

  @pytest.mark.parametrize("as_memoryview", [True, False])
  def test_the_emptiness_check_works_on_both(self, as_memoryview):
    """Le garde de la boucle principale : len() marche sur les deux, .size non."""
    assert len(FakeBuf(as_memoryview=as_memoryview).data) > 0

  def test_the_luminance_plane_ignores_the_stride_padding(self):
    y = sentineld.extract_y(FakeBuf(y_value=90))

    assert y.shape == (4, 8) and (y == 90).all()

  def test_a_grey_frame_stays_grey(self):
    ycbcr = sentineld.nv12_to_ycbcr(FakeBuf(y_value=90))

    assert ycbcr.shape == (4, 8, 3)
    assert (ycbcr[:, :, 0] == 90).all()          # luminance conservee
    assert (ycbcr[:, :, 1:] == 128).all()        # chrominance neutre : du gris

  def test_the_jpeg_round_trips_to_the_right_brightness(self):
    from io import BytesIO

    from PIL import Image

    jpeg = sentineld.encode_jpeg(FakeBuf(width=64, height=32, stride=80, y_value=200))
    img = Image.open(BytesIO(jpeg))

    assert jpeg[:2] == b"\xff\xd8"              # entete JPEG
    assert img.size == (64, 32)
    # relu en RGB, un gris de luminance 200 doit ressortir gris et clair
    r, g, b = img.convert("RGB").getpixel((32, 16))
    assert abs(r - 200) < 12 and abs(g - 200) < 12 and abs(b - 200) < 12



class TestState:
  """L'etat tient dans des fichiers : une cle de parametre ajoutee serait inconnue de
  la version precompilee qui tourne sur l'appareil, et ferait tomber le manager."""

  @pytest.fixture(autouse=True)
  def _paths(self, monkeypatch, tmp_path):
    monkeypatch.setattr(state, "ARMED_PATH", str(tmp_path / "sentry_armed"))
    monkeypatch.setattr(state, "MIN_VOLTAGE_PATH", str(tmp_path / "sentry_min_voltage"))
    monkeypatch.setattr(state, "STATUS_PATH", str(tmp_path / "sentry_status.json"))

  def test_arming_and_disarming(self):
    assert state.is_armed() is False
    state.arm()
    assert state.is_armed() is True
    state.disarm()
    assert state.is_armed() is False

  def test_disarming_twice_is_harmless(self):
    state.disarm()
    state.disarm()
    assert state.is_armed() is False

  def test_the_voltage_floor_defaults_to_the_openpilot_one(self):
    assert state.min_voltage_mv() == 11800

  def test_the_voltage_floor_can_be_raised_without_touching_the_code(self):
    with open(state.MIN_VOLTAGE_PATH, "w") as f:
      f.write("12200\n")
    assert state.min_voltage_mv() == 12200

  def test_a_broken_override_falls_back_to_the_default(self):
    with open(state.MIN_VOLTAGE_PATH, "w") as f:
      f.write("douze volts")
    assert state.min_voltage_mv() == 11800

  def test_the_status_round_trips(self):
    state.write_status({"armed": True, "events": 3})
    assert state.read_status() == {"armed": True, "events": 3}

  def test_a_missing_status_reads_empty(self):
    assert state.read_status() == {}


class TestBootEvent:
  """Debrancher le comma est la premiere chose a faire pour le faire taire."""

  @pytest.fixture(autouse=True)
  def _paths(self, monkeypatch, tmp_path):
    monkeypatch.setattr(state, "ARMED_PATH", str(tmp_path / "sentry_armed"))
    monkeypatch.setattr(state, "RUNNING_PATH", str(tmp_path / "sentry_running"))
    monkeypatch.setattr(state, "MIN_VOLTAGE_PATH", str(tmp_path / "sentry_min_voltage"))
    state.arm()
    self.root = str(tmp_path / "events")

  def test_the_watch_records_as_soon_as_it_starts(self):
    s = sentineld.Sentry(self.root)
    s.on_boot(0., power_cut=True)

    assert s.recorder.recording and s.recorder.triggers == ["boot"]

  def test_a_power_cut_is_told_apart_from_a_normal_start(self):
    s = sentineld.Sentry(self.root)
    s.on_boot(0., power_cut=True)
    meta = json.loads(open(os.path.join(s.recorder.path, "meta.json")).read())

    assert meta["power_cut"] is True and meta["triggers"] == ["boot"]

  def test_the_marker_survives_a_watch_that_was_cut_off(self):
    assert state.was_running() is False
    state.mark_running()
    assert state.was_running() is True        # coupure : le marqueur reste

  def test_a_deliberate_stop_leaves_no_marker(self):
    state.mark_running()
    state.clear_running()

    assert state.was_running() is False

  def test_clearing_twice_is_harmless(self):
    state.clear_running()
    state.clear_running()
    assert state.was_running() is False


class TestPowerTrace:
  """Le releve qui sert a dimensionner une batterie auxiliaire."""

  @pytest.fixture(autouse=True)
  def _paths(self, monkeypatch, tmp_path):
    self.path = str(tmp_path / "sentry_power.csv")
    monkeypatch.setattr(state, "ARMED_PATH", str(tmp_path / "sentry_armed"))
    monkeypatch.setattr(state, "MIN_VOLTAGE_PATH", str(tmp_path / "min_voltage"))
    state.arm()

  def _row(self, **kw):
    row = {"time": 1_789_900_000.0, "mono": 42.5, "boot": "3f2a1c9d", "voltage_mv": 12400,
           "power_w": 6.8, "som_w": 3.1, "recording": 0, "events": 4}
    row.update(kw)
    return row

  def test_the_file_starts_with_its_header(self):
    sentineld.append_trace(self._row(), self.path)
    lines = open(self.path).read().splitlines()

    assert lines[0].startswith("time,mono,boot,voltage_mv,power_w")
    assert lines[1] == "1789900000,42.5,3f2a1c9d,12400,6.80,3.10,0,4"

  def test_rows_accumulate(self):
    for i in range(5):
      sentineld.append_trace(self._row(mono=i * 30.), self.path)

    assert len(open(self.path).read().splitlines()) == 6   # entete plus cinq lignes

  def test_the_file_stays_bounded(self, monkeypatch):
    monkeypatch.setattr(sentineld, "TRACE_MAX_BYTES", 400)
    monkeypatch.setattr(sentineld, "TRACE_KEEP_LINES", 5)
    for i in range(200):
      sentineld.append_trace(self._row(mono=i * 30.), self.path)

    lines = open(self.path).read().splitlines()
    assert os.path.getsize(self.path) <= 400 * 2   # borne respectee apres coupe
    assert lines[0].startswith("time,")            # entete conservee
    assert lines[-1].split(",")[1] == "5970.0"     # la derniere mesure est gardee

  def test_the_row_carries_what_the_analysis_needs(self, tmp_path):
    s = sentineld.Sentry(str(tmp_path / "events"))
    s.voltage_mv, s.power_w, s.som_w = 12380, 7.2, 3.4
    row = s.trace_row()

    assert row["voltage_mv"] == 12380 and row["power_w"] == 7.2 and row["som_w"] == 3.4
    assert row["boot"] == sentineld.boot_id()[:8] and row["recording"] == 0

  def test_status_reports_the_power_too(self, tmp_path):
    s = sentineld.Sentry(str(tmp_path / "events"))
    s.power_w, s.som_w = 6.5, 3.0

    assert s.status()["power_w"] == 6.5 and s.status()["som_w"] == 3.0


class TestAutoArm:
  """Armement automatique a l'arret du moteur, reglable depuis l'ecran."""

  @pytest.fixture(autouse=True)
  def _paths(self, monkeypatch, tmp_path):
    monkeypatch.setattr(state, "ARMED_PATH", str(tmp_path / "sentry_armed"))
    monkeypatch.setattr(state, "AUTO_PATH", str(tmp_path / "sentry_auto"))

  def test_off_by_default(self):
    assert state.auto_enabled() is False
    assert state.should_auto_arm(ignition=False, ignition_prev=True) is False

  def test_the_switch_persists_both_ways(self):
    state.set_auto(True)
    assert state.auto_enabled() is True
    state.set_auto(False)
    assert state.auto_enabled() is False

  def test_it_arms_when_the_engine_stops(self):
    state.set_auto(True)
    assert state.should_auto_arm(ignition=False, ignition_prev=True) is True

  def test_it_does_nothing_while_the_engine_runs(self):
    state.set_auto(True)
    assert state.should_auto_arm(ignition=True, ignition_prev=True) is False
    assert state.should_auto_arm(ignition=True, ignition_prev=False) is False

  def test_it_does_not_re_arm_a_watch_that_stopped_itself(self):
    """Batterie basse : la veille s'arrete et se desarme. Sans front de contact, elle
    ne doit pas repartir en boucle et achever la batterie."""
    state.set_auto(True)
    assert state.should_auto_arm(ignition=False, ignition_prev=False) is False

  def test_it_does_not_arm_twice(self):
    state.set_auto(True)
    state.arm()
    assert state.should_auto_arm(ignition=False, ignition_prev=True) is False

  def test_manual_arming_still_works_with_the_switch_off(self):
    state.arm()
    assert state.is_armed() is True


class TestMotionCamera:
  """Savoir quelle camera a vu bouger : c'est elle qu'on ira regarder en premier."""

  @pytest.fixture(autouse=True)
  def _root(self, tmp_path, monkeypatch):
    monkeypatch.setattr(state, "ARMED_PATH", str(tmp_path / "armed"))
    monkeypatch.setattr(state, "MIN_VOLTAGE_PATH", str(tmp_path / "min_voltage"))
    state.arm()
    self.root = str(tmp_path / "events")

  def _meta(self, rec):
    return json.loads(open(os.path.join(rec.path, "meta.json")).read())

  def test_the_camera_is_noted_when_the_event_opens(self):
    rec = sentineld.EventRecorder(self.root)
    rec.trigger("motion", 0., {"motion_cameras": ["e"]})

    assert self._meta(rec)["motion_cameras"] == ["e"]

  def test_a_second_camera_is_added_during_the_event(self):
    rec = sentineld.EventRecorder(self.root)
    rec.trigger("motion", 0., {"motion_cameras": ["f"]})
    rec.trigger("motion", 1., {"motion_cameras": ["d"]})

    assert self._meta(rec)["motion_cameras"] == ["f", "d"]

  def test_the_same_camera_is_not_listed_twice(self):
    rec = sentineld.EventRecorder(self.root)
    for t in range(4):
      rec.trigger("motion", float(t), {"motion_cameras": ["f"]})

    assert self._meta(rec)["motion_cameras"] == ["f"]

  def test_motion_and_can_details_coexist(self):
    rec = sentineld.EventRecorder(self.root)
    rec.trigger("can", 0., {"can_buses": [0]})
    rec.trigger("motion", 1., {"motion_cameras": ["e"]})
    meta = self._meta(rec)

    assert meta["triggers"] == ["can", "motion"]
    assert meta["can_buses"] == [0] and meta["motion_cameras"] == ["e"]

  def test_the_detail_survives_the_closing(self):
    rec = sentineld.EventRecorder(self.root, post_event_s=1.)
    rec.trigger("motion", 0., {"motion_cameras": ["f", "e"]})
    path = rec.path
    rec.tick(10.)

    meta = json.loads(open(os.path.join(path, "meta.json")).read())
    assert meta["motion_cameras"] == ["f", "e"] and "closed" in meta

  def test_a_new_event_starts_with_a_clean_slate(self):
    rec = sentineld.EventRecorder(self.root, post_event_s=1.)
    rec.trigger("motion", 0., {"motion_cameras": ["d"]})
    rec.tick(10.)
    rec.trigger("motion", 20., {"motion_cameras": ["f"]})

    assert self._meta(rec)["motion_cameras"] == ["f"]

  def test_the_sentry_reports_the_camera_that_saw_it(self, tmp_path):
    s = sentineld.Sentry(str(tmp_path / "ev"))
    still, moving = frame(), frame()
    moving[20:90, 20:110] = 220
    s.on_frames({"e": (still, lambda: b"jpeg")}, 0.)
    s.on_frames({"e": (moving, lambda: b"jpeg")}, 1.)

    assert s.status()["motion_cameras"] == ["e"]


class TestMotionAgainstFoliage:
  """Le vent dans les arbres declenchait : il fait du grain disperse, pas une tache."""

  @pytest.fixture(autouse=True)
  def _defaults(self, monkeypatch, tmp_path):
    monkeypatch.setattr(state, "MOTION_DELTA_PATH", str(tmp_path / "delta"))
    monkeypatch.setattr(state, "MOTION_AREA_PATH", str(tmp_path / "area"))

  def _scattered(self, base, n=400, seed=0):
    """Des cellules isolees qui changent partout : le feuillage agite."""
    rng = np.random.default_rng(seed)
    out = base.copy()
    ys = rng.integers(0, base.shape[0], n)
    xs = rng.integers(0, base.shape[1], n)
    out[ys, xs] = 255
    return out

  def test_scattered_change_does_not_trigger(self):
    d = sentineld.MotionDetector()
    base = frame(width=1344, height=760)
    d.update(base)

    assert d.update(self._scattered(base)) is False

  def test_a_solid_shape_still_triggers(self):
    d = sentineld.MotionDetector()
    base = frame(width=1344, height=760)
    d.update(base)
    person = base.copy()
    person[300:600, 400:700] = 220        # une silhouette proche

    assert d.update(person) is True

  def test_the_grouped_score_is_far_below_the_scattered_one(self):
    """C'est ce rapport qui separe le vent d'une personne : mesure 0,44 % contre 14,5 %."""
    base = frame(width=1344, height=760)
    d1, d2 = sentineld.MotionDetector(), sentineld.MotionDetector()
    d1.score(base), d2.score(base)
    person = base.copy()
    person[300:600, 400:700] = 220

    assert d1.score(self._scattered(base)) < 0.2 * d2.score(person)

  def test_the_threshold_is_tunable_without_touching_the_code(self, tmp_path):
    with open(state.MOTION_AREA_PATH, "w") as f:
      f.write("0.5\n")     # exige que la moitie de l'image bouge
    with open(state.MOTION_DELTA_PATH, "w") as f:
      f.write("30\n")

    d = sentineld.MotionDetector()
    assert d.area == 0.5 and d.delta == 30

    base = frame(width=1344, height=760)
    d.update(base)
    person = base.copy()
    person[300:600, 400:700] = 220
    assert d.update(person) is False      # la silhouette ne suffit plus, comme demande

  def test_a_broken_setting_falls_back_to_the_default(self):
    with open(state.MOTION_AREA_PATH, "w") as f:
      f.write("beaucoup")

    assert state.motion_area() == state.DEFAULT_MOTION_AREA
    assert state.motion_delta() == state.DEFAULT_MOTION_DELTA

  def test_a_global_brightness_change_still_does_not_trigger(self):
    d = sentineld.MotionDetector()
    d.update(frame(width=1344, height=760, value=100))

    assert d.update(frame(width=1344, height=760, value=160)) is False


class TestCarSidePower:
  """Le capteur de puissance globale n'existe pas sur mici : on passe par le panda."""

  def test_the_row_carries_the_car_side_power(self, tmp_path):
    s = sentineld.Sentry(str(tmp_path / "ev"))
    s.voltage_mv, s.current_ma = 12460, 205
    s.power_w = s.voltage_mv * s.current_ma / 1e6

    assert abs(s.trace_row()["power_w"] - 2.56) < 0.01

  def test_the_current_starts_at_zero(self, tmp_path):
    assert sentineld.Sentry(str(tmp_path / "ev")).current_ma == 0.


class TestFinish:
  """Fermeture propre : ce qui distingue un arret ordonne d'une coupure de courant."""

  @pytest.fixture(autouse=True)
  def _paths(self, monkeypatch, tmp_path):
    monkeypatch.setattr(state, "ARMED_PATH", str(tmp_path / "armed"))
    monkeypatch.setattr(state, "RUNNING_PATH", str(tmp_path / "running"))
    monkeypatch.setattr(state, "STATUS_PATH", str(tmp_path / "status.json"))
    monkeypatch.setattr(state, "MIN_VOLTAGE_PATH", str(tmp_path / "min_voltage"))
    state.arm()
    state.mark_running()
    self.sentry = sentineld.Sentry(str(tmp_path / "events"))

  def test_an_ordered_stop_stays_armed_and_leaves_no_marker(self):
    """Redemarrage demande : la veille doit reprendre apres, sans crier a la coupure."""
    self.sentry.stop_reason = "stopped"

    assert sentineld.finish(self.sentry) is False
    assert state.is_armed() is True and state.was_running() is False

  def test_the_engine_starting_disarms(self):
    self.sentry.stop_reason = "ignition"
    sentineld.finish(self.sentry)

    assert state.is_armed() is False and state.was_running() is False

  def test_a_flat_battery_disarms_and_asks_for_shutdown(self):
    self.sentry.stop_reason = "voltage"

    assert sentineld.finish(self.sentry) is True
    assert state.is_armed() is False

  def test_the_open_event_is_closed(self):
    self.sentry.recorder.trigger("motion", 0., {"motion_cameras": ["f"]})
    path = self.sentry.recorder.path
    self.sentry.stop_reason = "stopped"
    sentineld.finish(self.sentry)

    assert not self.sentry.recorder.recording
    assert "closed" in json.loads(open(os.path.join(path, "meta.json")).read())

  def test_the_status_is_published_on_the_way_out(self):
    self.sentry.stop_reason = "ignition"
    sentineld.finish(self.sentry)

    assert state.read_status()["stop_reason"] == "ignition"
