"""Phase C cadence-braking baseline.

A rider pumping the brake lever is modelled as a square-wave chop of the
human V_pwm_cmd at ``freq`` Hz with ``duty`` on-fraction. During the
*on* half of each cycle ``V_pwm = V_pwm_cmd``; during the *off* half
``V_pwm = 0`` and the motor coasts under its viscous load, which lets
the hydraulic clamp bleed down through ``τ_hyd``.

Inputs: ``V_pwm_cmd``. Outputs: ``V_pwm``, ``cadence_on`` (diagnostic
0/1 so run_comparison can shade the on-segments).
"""

from __future__ import annotations

from ..block import Block


class CadenceController(Block):
    name = "cadence"
    inputs = ["V_pwm_cmd"]
    outputs = ["V_pwm", "cadence_on"]

    def __init__(self, freq: float, duty: float):
        self.freq = float(freq)
        self.duty = float(duty)
        if not 0.0 <= self.duty <= 1.0:
            raise ValueError(f"duty must be in [0, 1], got {duty}")
        if self.freq <= 0.0:
            raise ValueError(f"freq must be > 0, got {freq}")
        self.period = 1.0 / self.freq

    def step(self, t, u):
        V_cmd = u.get("V_pwm_cmd", 0.0)
        phase = (t % self.period) / self.period
        on = 1.0 if phase < self.duty else 0.0
        return {"V_pwm": V_cmd * on, "cadence_on": on}
