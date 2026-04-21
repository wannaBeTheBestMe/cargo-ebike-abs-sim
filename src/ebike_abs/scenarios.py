"""Scenario builders — helpers that assemble a :class:`Simulator` from a
parsed config dict. Kept outside the ``scripts/`` folder so the same
builder is shared by the runner script and the integration tests.
"""

from __future__ import annotations

import tomllib
from pathlib import Path

from .blocks.brake import BrakeTorqueComputation, PrescribedClamp
from .blocks.normal_load import NormalLoad
from .blocks.slip import SlipRatioTrue
from .blocks.tire import BrushTireModel
from .blocks.vehicle import VehicleTranslation
from .blocks.wheel import FrontWheelRotation, RearWheelKinematics
from .simulator import Simulator


def load_config(path: Path | str) -> dict:
    with open(path, "rb") as f:
        return tomllib.load(f)


def build_phase_a_panic_stop(cfg: dict) -> Simulator:
    """Full Phase A plant + prescribed-clamp actuator stand-in."""
    v0 = cfg["scenario"]["v0"]
    R_f = cfg["vehicle"]["R_f"]
    blocks = [
        VehicleTranslation(m=cfg["vehicle"]["m"], v0=v0),
        FrontWheelRotation(I_f=cfg["vehicle"]["I_f"], R_f=R_f, omega0=v0 / R_f),
        RearWheelKinematics(R_r=cfg["vehicle"]["R_r"]),
        PrescribedClamp(
            F_peak=cfg["scenario"]["prescribed_clamp"]["F_peak"],
            t_rise=cfg["scenario"]["prescribed_clamp"]["t_rise"],
        ),
        NormalLoad(
            m=cfg["vehicle"]["m"],
            g=cfg["vehicle"]["g"],
            a=cfg["vehicle"]["a"],
            h=cfg["vehicle"]["h"],
            L=cfg["vehicle"]["L"],
        ),
        SlipRatioTrue(R_f=R_f, v_epsilon=cfg["scenario"]["v_epsilon"]),
        BrushTireModel(mu_peak=cfg["tire"]["mu_peak"], C_x=cfg["tire"]["C_x"]),
        BrakeTorqueComputation(
            mu_pad=cfg["brake"]["mu_pad"],
            r_eff=cfg["brake"]["r_eff"],
            n_pads=cfg["brake"]["n_pads"],
        ),
    ]
    return Simulator(blocks, dt=cfg["scenario"]["dt"])
