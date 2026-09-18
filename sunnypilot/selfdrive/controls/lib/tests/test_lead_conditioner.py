import cereal.messaging as messaging

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.lead_conditioner import LeadConditioner


def _radar(d_rel, v_lead, v_ego, a_model=0., prob=1.0, status=True, radar=False):
  msg = messaging.new_message('radarState')
  lead = msg.radarState.leadOne
  lead.status = status
  lead.modelProb = prob
  lead.radar = radar
  lead.dRel = d_rel
  lead.vLead = lead.vLeadK = v_lead
  lead.vRel = v_lead - v_ego
  lead.aLeadK = a_model
  lead.aLeadTau = 0.3
  return msg.radarState.as_reader()


class TestLeadConditioner:

  def setup_method(self):
    self.lc = LeadConditioner()

  def _track(self, seconds, v_lead=15., a_lead=0., v_ego=15., d_rel=30., a_model=0.):
    out = None
    for _ in range(int(round(seconds / DT_MDL))):
      out = self.lc.update(_radar(d_rel, v_lead, v_ego, a_model=a_model), v_ego)
      v_lead = max(v_lead + a_lead * DT_MDL, 0.)
      d_rel += (v_lead - v_ego) * DT_MDL
    return out

  def test_radar_lead_untouched(self):
    rs = _radar(30., 15., 15., a_model=-0.1, radar=True)
    assert self.lc.update(rs, 15.) is rs

  def test_untouched_before_track_min(self):
    rs = _radar(30., 15., 15., a_model=0.05)
    assert self.lc.update(rs, 15.) is rs

  def test_close_braking_lead_uses_derived_accel(self):
    # the vision model reports ~5% of the real deceleration
    out = self._track(3.0, a_lead=-2.0, d_rel=15., a_model=-0.1)
    assert out.leadOne.aLeadK < -1.5
    assert abs(out.leadOne.aLeadTau - 0.3) < 1e-6  # aLeadTau left untouched

  def test_far_lead_keeps_model_accel(self):
    # vLead is too noisy far away for a derivative: the model estimate is kept as is
    out = self._track(3.0, a_lead=-2.0, d_rel=80., a_model=-0.1)
    assert abs(out.leadOne.aLeadK - -0.1) < 1e-6

  def test_constant_close_lead_near_zero(self):
    out = self._track(3.0, d_rel=15., a_model=0.05)
    assert abs(out.leadOne.aLeadK) < 0.1

  def test_short_track_not_held(self):
    # a 1.2 s confident detection (phantom) must not be extrapolated once it disappears
    self._track(1.2, v_lead=0., v_ego=14., d_rel=30.)
    out = self.lc.update(_radar(0., 0., 14., prob=0., status=False), 14.)
    assert not out.leadOne.status

  def test_lost_lead_held_then_released(self):
    self._track(3.0, v_lead=10., v_ego=14., d_rel=30.)
    lost = _radar(0., 0., 14., prob=0., status=False)
    held = [self.lc.update(lost, 14.) for _ in range(int(round(LeadConditioner.HOLD_T / DT_MDL)))]
    assert all(o.leadOne.status for o in held)
    assert held[-1].leadOne.dRel < held[0].leadOne.dRel  # still closing on the held lead
    assert not self.lc.update(lost, 14.).leadOne.status
