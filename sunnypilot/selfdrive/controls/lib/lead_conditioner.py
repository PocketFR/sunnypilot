#!/usr/bin/env python3
"""
Conditionnement de la cible vision avant le MPC longitudinal.

Mesure sur les logs (Ioniq PHEV, radarUnavailable, cible = modele vision) :
l'acceleration de cible fournie par le modele (lead.a[0] -> aLeadK) ne capte
que 3 a 14 % de la deceleration reelle (correlation 0.22-0.54). Pendant de
vrais freinages de la cible a -1.35 / -1.73 / -3.29 m/s2, aLeadK valait en
moyenne -0.19 / -0.03 / -0.09. Le MPC extrapole la cible avec cette valeur :
il la croit a vitesse constante, freine juste assez pour s'aligner sur sa
vitesse du moment, relache, puis refreine quand elle a encore ralenti.

Par ailleurs, quand la cible disparait (17 pertes en 3 min de suivi, mediane
0.35 s), long_mpc.process_lead la remplace par une voiture fictive a 50 m qui
s'eloigne a v_ego + 10 m/s : le MPC relache pendant la perte.

Ce module corrige l'entree du MPC, pas sa sortie :
  1. aLeadK : on estime l'acceleration de la cible par la derivee filtree de sa
     vitesse et on retient la plus freinante des deux estimations. La derivee
     n'est fiable que de pres (bruit de vLead mesure : 0.14 m/s a 8-15 m, 0.38 a
     15-30 m, 0.71 au-dela) : son poids passe continument de 1 sous W_D_FULL a
     0 au-dela de W_D_ZERO. Tout est continu : un premier essai a seuils
     (estimation a -0.3 m/s2, aLeadTau bascule entre 1.5 et 0) faisait osciller
     le MPC sur les cibles lointaines et ajoutait des cycles relache/refrein ;
  2. memoire : une cible suivie avec confiance depuis HOLD_MIN_TRACK_T qui
     disparait est extrapolee pendant HOLD_T au lieu d'etre remplacee par une
     voiture fictive.

aLeadTau n'est pas modifie.

Seules les cibles vision sont touchees (lead.radar == False).
"""
import numpy as np

from openpilot.common.realtime import DT_MDL


class LeadConditioner:
  PROB_MIN = 0.7          # confiance mini pour utiliser l'estimation / memoriser
  TRACK_MIN_T = 0.5       # s de suivi continu avant de s'en servir (anti-fantome)
  HOLD_T = 1.0            # s de memoire apres une perte de cible
  HOLD_MIN_TRACK_T = 2.0  # s de suivi avant d'autoriser la memoire : en simulation, un
                          # fantome de 1.2 s a prob 0.9 memorise 1 s de plus coutait
                          # 5 km/h de freinage inutile ; une vraie cible en file est
                          # suivie bien plus longtemps avant une perte
  V_TAU = 0.15            # s, filtre de vLead avant derivation
  A_TAU = 0.4             # s, filtre de l'acceleration derivee
  A_MIN, A_MAX = -5.0, 2.0
  W_D_FULL = 20.          # m, poids plein de l'estimation derivee en dessous
  W_D_ZERO = 40.          # m, poids nul au-dela

  def __init__(self, dt: float = DT_MDL):
    self.dt = dt
    self.hold_frames = int(round(self.HOLD_T / dt))
    self.reset()

  def reset(self) -> None:
    self.track_t = 0.
    self.hold_left = 0
    self.v_f = 0.
    self.a_f = 0.
    self.d = self.v = self.a = self.prob = self.tau = 0.

  def update(self, radar_state, v_ego: float):
    lead = radar_state.leadOne
    if lead.radar:
      self.reset()
      return radar_state

    confident = bool(lead.status) and lead.modelProb >= self.PROB_MIN
    if confident:
      if self.track_t == 0.:
        self.v_f, self.a_f = lead.vLead, 0.
      v_prev = self.v_f
      self.v_f += self.dt / (self.V_TAU + self.dt) * (lead.vLead - self.v_f)
      a_raw = float(np.clip((self.v_f - v_prev) / self.dt, self.A_MIN, self.A_MAX))
      self.a_f += self.dt / (self.A_TAU + self.dt) * (a_raw - self.a_f)
      self.track_t += self.dt
      self.d, self.v, self.prob, self.tau = lead.dRel, lead.vLead, lead.modelProb, lead.aLeadTau
      if self.track_t < self.TRACK_MIN_T:
        return radar_state

      w = float(np.interp(lead.dRel, [self.W_D_FULL, self.W_D_ZERO], [1., 0.]))
      a_lead = (1. - w) * lead.aLeadK + w * min(lead.aLeadK, self.a_f)
      self.a = a_lead
      self.hold_left = self.hold_frames if self.track_t >= self.HOLD_MIN_TRACK_T else 0
      out = radar_state.as_builder()
      out.leadOne.aLeadK = float(a_lead)
      return out

    if self.hold_left > 0 and not lead.status:
      # cible perdue : on la garde, en extrapolant son mouvement
      self.hold_left -= 1
      self.v = max(self.v + self.a * self.dt, 0.)
      self.d += (self.v - v_ego) * self.dt
      out = radar_state.as_builder()
      lo = out.leadOne
      lo.status = True
      lo.dRel = float(max(self.d, 0.))
      lo.vLead = lo.vLeadK = float(self.v)
      lo.vRel = float(self.v - v_ego)
      lo.aLeadK = float(self.a)
      lo.aLeadTau = float(self.tau)
      lo.modelProb = float(self.prob)
      return out

    self.track_t = 0.
    self.hold_left = 0
    return radar_state
