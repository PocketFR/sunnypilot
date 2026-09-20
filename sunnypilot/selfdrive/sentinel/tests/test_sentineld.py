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

  def __init__(self, width=8, height=4, stride=16, y_value=90):
    self.width, self.height, self.stride = width, height, stride
    self.uv_offset = stride * height
    uv_height = ((height // 2) + 15) // 16 * 16
    data = np.full(self.uv_offset + stride * uv_height, 128, dtype=np.uint8)  # U = V = 128 : gris
    y = data[:self.uv_offset].reshape((-1, stride))
    y[:height, :width] = y_value
    y[:height, width:] = 7  # bourrage de ligne : ne doit jamais ressortir
    self.data = data


class TestFrameExtraction:
  def test_the_luminance_plane_ignores_the_stride_padding(self):
    y = sentineld.extract_y(FakeBuf(y_value=90))

    assert y.shape == (4, 8) and (y == 90).all()

  def test_a_grey_frame_converts_to_grey(self):
    rgb = sentineld.nv12_to_rgb(FakeBuf(y_value=90))

    assert rgb.shape == (4, 8, 3)
    assert abs(int(rgb[0, 0, 0]) - 90) <= 1 and abs(int(rgb[0, 0, 2]) - 90) <= 1

  def test_the_jpeg_round_trips(self):
    from io import BytesIO

    from PIL import Image

    jpeg = sentineld.encode_jpeg(sentineld.nv12_to_rgb(FakeBuf(width=64, height=32, stride=80, y_value=200)))

    assert jpeg[:2] == b"\xff\xd8"  # entete JPEG
    assert Image.open(BytesIO(jpeg)).size == (64, 32)



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
