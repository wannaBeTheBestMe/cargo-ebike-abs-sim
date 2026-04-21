"""Phase B human-baseline brake controller.

The rider is modelled as squeezing the motor-actuated brake lever with a
linear $V_{\text{pwm}}(t)$ ramp from 0 to ``V_hold`` over ``t_rise``,
then holding. No closed-loop reaction to wheel lockup — that's the
point of a *baseline*. Phase C will layer the ABS FSM on top of this
profile (gating the human command) and the cadence controller (square
-wave chopping it).

Inputs: none. Output: ``V_pwm`` into the motor-actuator chain.
"""

from __future__ import annotations

from ..block import Block


class HumanBrakeController(Block):
    name = "human"
    inputs: list[str] = []
    outputs = ["V_pwm_cmd"]

    def __init__(self, V_hold: float, t_rise: float):
        self.V_hold = float(V_hold)
        self.t_rise = float(t_rise)

    def step(self, t, u):
        frac = 1.0 if self.t_rise <= 0.0 else min(1.0, max(0.0, t / self.t_rise))
        return {"V_pwm_cmd": self.V_hold * frac}


class Passthrough(Block):
    """Copy ``V_pwm_cmd`` → ``V_pwm`` unchanged.

    Used by the Phase B human-only scenario where no ABS / cadence gate
    sits between the rider and the actuator.
    """

    name = "gate_passthrough"
    inputs = ["V_pwm_cmd"]
    outputs = ["V_pwm"]

    def step(self, t, u):
        return {"V_pwm": u.get("V_pwm_cmd", 0.0)}
