"""Scenario builders — helpers that assemble a :class:`Simulator` from a
parsed config dict. Kept outside the ``scripts/`` folder so the same
builder is shared by the runner script and the integration tests.
"""

from __future__ import annotations

import tomllib
from pathlib import Path

from .blocks.actuator import MotorActuator
from .blocks.brake import BrakeTorqueComputation, PrescribedClamp
from .blocks.normal_load import NormalLoad
from .blocks.sensor import HallSensor, WheelSpeedEstimator
from .blocks.slip import SlipRatioEstimated, SlipRatioTrue
from .blocks.tire import DugoffTireModel
from .blocks.vehicle import VehicleTranslation
from .blocks.wheel import FrontWheelRotation, RearWheelKinematics
from .control.abs_fsm import ABSController
from .control.cadence import CadenceController
from .control.human import HumanBrakeController, Passthrough
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
        DugoffTireModel(mu_peak=cfg["tire"]["mu_peak"], C_x=cfg["tire"]["C_x"]),
        BrakeTorqueComputation(
            mu_pad=cfg["brake"]["mu_pad"],
            r_eff=cfg["brake"]["r_eff"],
            n_pads=cfg["brake"]["n_pads"],
        ),
    ]
    return Simulator(blocks, dt=cfg["scenario"]["dt"])


def build_phase_b_panic_stop(cfg: dict) -> Simulator:
    """Phase B plant: human V_pwm ramp → motor actuator → brake → plant,
    with the Hall + estimator + estimated-slip chain running in parallel.
    Drops the prescribed-clamp stand-in from Phase A.
    """
    v0 = cfg["scenario"]["v0"]
    R_f = cfg["vehicle"]["R_f"]
    R_r = cfg["vehicle"]["R_r"]
    dt = cfg["scenario"]["dt"]
    v_eps = cfg["scenario"]["v_epsilon"]
    act_cfg = cfg["actuator"]
    blocks = [
        VehicleTranslation(m=cfg["vehicle"]["m"], v0=v0),
        FrontWheelRotation(I_f=cfg["vehicle"]["I_f"], R_f=R_f, omega0=v0 / R_f),
        RearWheelKinematics(R_r=R_r),
        HumanBrakeController(
            V_hold=cfg["controller"]["human"]["V_hold"],
            t_rise=cfg["controller"]["human"]["t_rise"],
        ),
        Passthrough(),
        MotorActuator(
            L_m=act_cfg["L_m"],
            R_m=act_cfg["R_m"],
            K_e=act_cfg["K_e"],
            K_t=act_cfg["K_t"],
            J_m=act_cfg["J_m"],
            b_m=act_cfg["b_m"],
            r_lever=act_cfg["r_lever"],
            A_master=act_cfg["A_master"],
            A_caliper=act_cfg["A_caliper"],
            tau_hyd=act_cfg["tau_hyd"],
            V_pwm_max=act_cfg["V_pwm_max"],
        ),
        NormalLoad(
            m=cfg["vehicle"]["m"],
            g=cfg["vehicle"]["g"],
            a=cfg["vehicle"]["a"],
            h=cfg["vehicle"]["h"],
            L=cfg["vehicle"]["L"],
        ),
        SlipRatioTrue(R_f=R_f, v_epsilon=v_eps),
        DugoffTireModel(mu_peak=cfg["tire"]["mu_peak"], C_x=cfg["tire"]["C_x"]),
        BrakeTorqueComputation(
            mu_pad=cfg["brake"]["mu_pad"],
            r_eff=cfg["brake"]["r_eff"],
            n_pads=cfg["brake"]["n_pads"],
        ),
        HallSensor(N_hall=cfg["sensor"]["N_hall"]),
        WheelSpeedEstimator(
            N_hall=cfg["sensor"]["N_hall"],
            lpf_fc=cfg["estimator"]["lpf_fc"],
            ma_window=cfg["estimator"]["ma_window"],
            dt=dt,
            omega0=v0 / R_f,
        ),
        SlipRatioEstimated(R_f=R_f, R_r=R_r, v_epsilon=v_eps),
    ]
    return Simulator(blocks, dt=dt)


def build_phase_c_cadence_panic_stop(cfg: dict) -> Simulator:
    """Phase C cadence-baseline run: human V_pwm command is square-wave
    chopped at ``controller.cadence.freq`` Hz with ``duty`` duty cycle
    before it hits the motor actuator. Same plant + estimator chain as
    the ABS scenario so the two can be diffed directly.
    """
    v0 = cfg["scenario"]["v0"]
    R_f = cfg["vehicle"]["R_f"]
    R_r = cfg["vehicle"]["R_r"]
    dt = cfg["scenario"]["dt"]
    v_eps = cfg["scenario"]["v_epsilon"]
    act_cfg = cfg["actuator"]
    cad_cfg = cfg["controller"]["cadence"]
    blocks = [
        VehicleTranslation(m=cfg["vehicle"]["m"], v0=v0),
        FrontWheelRotation(I_f=cfg["vehicle"]["I_f"], R_f=R_f, omega0=v0 / R_f),
        RearWheelKinematics(R_r=R_r),
        HumanBrakeController(
            V_hold=cfg["controller"]["human"]["V_hold"],
            t_rise=cfg["controller"]["human"]["t_rise"],
        ),
        CadenceController(freq=cad_cfg["freq"], duty=cad_cfg["duty"]),
        MotorActuator(
            L_m=act_cfg["L_m"],
            R_m=act_cfg["R_m"],
            K_e=act_cfg["K_e"],
            K_t=act_cfg["K_t"],
            J_m=act_cfg["J_m"],
            b_m=act_cfg["b_m"],
            r_lever=act_cfg["r_lever"],
            A_master=act_cfg["A_master"],
            A_caliper=act_cfg["A_caliper"],
            tau_hyd=act_cfg["tau_hyd"],
            V_pwm_max=act_cfg["V_pwm_max"],
        ),
        NormalLoad(
            m=cfg["vehicle"]["m"],
            g=cfg["vehicle"]["g"],
            a=cfg["vehicle"]["a"],
            h=cfg["vehicle"]["h"],
            L=cfg["vehicle"]["L"],
        ),
        SlipRatioTrue(R_f=R_f, v_epsilon=v_eps),
        DugoffTireModel(mu_peak=cfg["tire"]["mu_peak"], C_x=cfg["tire"]["C_x"]),
        BrakeTorqueComputation(
            mu_pad=cfg["brake"]["mu_pad"],
            r_eff=cfg["brake"]["r_eff"],
            n_pads=cfg["brake"]["n_pads"],
        ),
        HallSensor(N_hall=cfg["sensor"]["N_hall"]),
        WheelSpeedEstimator(
            N_hall=cfg["sensor"]["N_hall"],
            lpf_fc=cfg["estimator"]["lpf_fc"],
            ma_window=cfg["estimator"]["ma_window"],
            dt=dt,
            omega0=v0 / R_f,
        ),
        SlipRatioEstimated(R_f=R_f, R_r=R_r, v_epsilon=v_eps),
    ]
    return Simulator(blocks, dt=dt)


def build_phase_c_abs_panic_stop(cfg: dict) -> Simulator:
    """Phase C ABS run: human command passes through the ABS FSM, which
    gates V_pwm to the motor actuator based on the estimator chain's
    λ̂_f / ω̇̂_f / v̂ feedback. ABS feedback signals are read lag-1 from
    the previous step's persistent signals (same pattern as N_f↔a_x)
    because the sensor chain sits downstream of the plant.
    """
    v0 = cfg["scenario"]["v0"]
    R_f = cfg["vehicle"]["R_f"]
    R_r = cfg["vehicle"]["R_r"]
    dt = cfg["scenario"]["dt"]
    v_eps = cfg["scenario"]["v_epsilon"]
    act_cfg = cfg["actuator"]
    abs_cfg = cfg["controller"]["abs"]
    blocks = [
        VehicleTranslation(m=cfg["vehicle"]["m"], v0=v0),
        FrontWheelRotation(I_f=cfg["vehicle"]["I_f"], R_f=R_f, omega0=v0 / R_f),
        RearWheelKinematics(R_r=R_r),
        HumanBrakeController(
            V_hold=cfg["controller"]["human"]["V_hold"],
            t_rise=cfg["controller"]["human"]["t_rise"],
        ),
        ABSController(
            lambda_on=abs_cfg["lambda_on"],
            lambda_off=abs_cfg["lambda_off"],
            omega_dot_trig=abs_cfg["omega_dot_trig"],
            v_cutoff=abs_cfg["v_cutoff"],
            dump_dwell=abs_cfg.get("dump_dwell", 0.050),
        ),
        MotorActuator(
            L_m=act_cfg["L_m"],
            R_m=act_cfg["R_m"],
            K_e=act_cfg["K_e"],
            K_t=act_cfg["K_t"],
            J_m=act_cfg["J_m"],
            b_m=act_cfg["b_m"],
            r_lever=act_cfg["r_lever"],
            A_master=act_cfg["A_master"],
            A_caliper=act_cfg["A_caliper"],
            tau_hyd=act_cfg["tau_hyd"],
            V_pwm_max=act_cfg["V_pwm_max"],
        ),
        NormalLoad(
            m=cfg["vehicle"]["m"],
            g=cfg["vehicle"]["g"],
            a=cfg["vehicle"]["a"],
            h=cfg["vehicle"]["h"],
            L=cfg["vehicle"]["L"],
        ),
        SlipRatioTrue(R_f=R_f, v_epsilon=v_eps),
        DugoffTireModel(mu_peak=cfg["tire"]["mu_peak"], C_x=cfg["tire"]["C_x"]),
        BrakeTorqueComputation(
            mu_pad=cfg["brake"]["mu_pad"],
            r_eff=cfg["brake"]["r_eff"],
            n_pads=cfg["brake"]["n_pads"],
        ),
        HallSensor(N_hall=cfg["sensor"]["N_hall"]),
        WheelSpeedEstimator(
            N_hall=cfg["sensor"]["N_hall"],
            lpf_fc=cfg["estimator"]["lpf_fc"],
            ma_window=cfg["estimator"]["ma_window"],
            dt=dt,
            omega0=v0 / R_f,
        ),
        SlipRatioEstimated(R_f=R_f, R_r=R_r, v_epsilon=v_eps),
    ]
    return Simulator(blocks, dt=dt)
