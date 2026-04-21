"""SlipRatioEstimated agrees with SlipRatioTrue when sensor noise is off."""

from __future__ import annotations

import numpy as np

from ebike_abs.blocks.sensor import HallSensor, WheelSpeedEstimator
from ebike_abs.blocks.slip import SlipRatioEstimated, SlipRatioTrue
from ebike_abs.blocks.vehicle import VehicleTranslation
from ebike_abs.blocks.wheel import FrontWheelRotation, RearWheelKinematics
from ebike_abs.simulator import Simulator


def test_slip_estimate_matches_truth_no_noise():
    # Coast-down (no brake, no tire force) with the Hall + estimator chain
    # running. With λ_r = 0 and no Hall noise, λ̂_f should track λ_f^true
    # within the v_ε/v bound everywhere the vehicle is well above v_ε.
    R_f = R_r = 0.33
    v0 = 8.0
    dt = 1.0e-4
    v_eps = 0.1
    blocks = [
        VehicleTranslation(m=120.0, v0=v0),
        FrontWheelRotation(I_f=0.12, R_f=R_f, omega0=v0 / R_f),
        RearWheelKinematics(R_r=R_r),
        HallSensor(N_hall=20),
        WheelSpeedEstimator(N_hall=20, lpf_fc=50.0, ma_window=4, dt=dt),
        SlipRatioTrue(R_f=R_f, v_epsilon=v_eps),
        SlipRatioEstimated(R_f=R_f, R_r=R_r, v_epsilon=v_eps),
    ]
    sim = Simulator(blocks, dt=dt)
    rows = sim.run(0.5)
    # Skip the transient before the LPF + MA have converged. At ω≈24 rad/s
    # edges come every ~13 ms, so 200 ms gives ~15 edges — plenty for a
    # 4-sample MA to settle.
    t_warm = 0.20
    max_err = 0.0
    for r in rows:
        if r["t"] < t_warm:
            continue
        v = r["v"]
        if v < 10.0 * v_eps:
            continue  # away from the denominator clamp
        err = abs(r["lambda_f_hat"] - r["lambda_f_true"])
        max_err = max(max_err, err)
    # Bound: both sides see zero physical slip; the residual is filter lag
    # at constant ω. 5 × 10⁻³ is generous — the measured residual is
    # a few × 10⁻⁴ after the MA fills.
    assert max_err < 5e-3, f"max |λ̂ − λ_true| = {max_err:.6e}"
    # Sanity check: λ_true ≈ 0 throughout.
    lam_true = np.array([r["lambda_f_true"] for r in rows])
    assert abs(lam_true).max() < 1e-6
