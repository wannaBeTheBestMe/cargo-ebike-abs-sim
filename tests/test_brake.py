"""Brake torque block and prescribed clamp source."""

from __future__ import annotations

import pytest

from ebike_abs.blocks.brake import BrakeTorqueComputation, PrescribedClamp


def test_brake_torque_linear_in_clamp():
    brake = BrakeTorqueComputation(mu_pad=0.4, r_eff=0.14, n_pads=2)
    T = brake.step(0.0, {"F_clamp": 1000.0})["T_b"]
    assert T == pytest.approx(0.4 * 1000.0 * 0.14 * 2, rel=1e-12)


def test_brake_torque_zero_when_clamp_zero():
    brake = BrakeTorqueComputation(mu_pad=0.4, r_eff=0.14, n_pads=2)
    assert brake.step(0.0, {"F_clamp": 0.0})["T_b"] == 0.0


def test_prescribed_clamp_starts_at_zero():
    clamp = PrescribedClamp(F_peak=2000.0, t_rise=0.15)
    assert clamp.step(0.0, {})["F_clamp"] == 0.0


def test_prescribed_clamp_halfway_is_half_peak():
    clamp = PrescribedClamp(F_peak=2000.0, t_rise=0.2)
    assert clamp.step(0.1, {})["F_clamp"] == pytest.approx(1000.0, rel=1e-12)


def test_prescribed_clamp_saturates_after_trise():
    clamp = PrescribedClamp(F_peak=2000.0, t_rise=0.15)
    assert clamp.step(0.3, {})["F_clamp"] == 2000.0
    assert clamp.step(5.0, {})["F_clamp"] == 2000.0
