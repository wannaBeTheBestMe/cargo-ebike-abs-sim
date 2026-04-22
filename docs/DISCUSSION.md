# Discussion

All numbers below are the live output of `python scripts/run_comparison.py`
on `configs/default.toml` (30 km/h panic stop, dry asphalt, μ_peak = 0.9).

| controller       | distance [m] | t_stop [s] | peak \|λ\| | peak \|a_x\| [m/s²] | lock-frac | \|λ\|>0.5 |
|------------------|-------------:|-----------:|-----------:|--------------------:|----------:|----------:|
| human (Phase B)  |         4.71 |       1.02 |       1.02 |                8.83 |      72 % |      76 % |
| cadence (2 Hz)   |         7.81 |       1.73 |       1.02 |                8.83 |      18 % |      25 % |
| ABS FSM          |         7.22 |       1.52 |       1.02 |                8.83 |      12 % |      22 % |

## 6.1 Why ABS doesn't win here — and when it would

On dry asphalt the model's Dugoff tire lumps peak and sliding μ into a single
value, so a locked wheel still slides at ≈ μ·N_f. The locked human baseline
is therefore the *distance-optimal* stop at **4.71 m** — it simply surrenders
all front-wheel steering to get there. Cadence and ABS trade ≈ 3 m of stop
distance to keep the wheel rolling: cadence **7.81 m**, ABS **7.22 m**. ABS
edges cadence by ≈ 7.5 % because its DUMP/HOLD schedule closes the loop on
λ, where cadence dumps pressure on a fixed 50 % duty cycle regardless of
whether the wheel is actually in the unstable regime. ABS also spends the
least time at dangerous slip (|λ|>0.5: 22 % vs 25 %).

This is the expected null result for a cargo e-bike on high-μ. ABS's real
case is made on **low-μ surfaces** where sliding μ falls well below peak μ:
wet asphalt, gravel, paint, manhole covers. Those scenarios aren't in
`configs/default.toml`; adding a μ_peak = 0.35 scenario is the natural Phase
D follow-up and is the main unfinished business of the study. The honest
high-μ conclusion is that a cadence-braking rider, an ABS, and a locked-slide
human end up within one rider-reaction-time of each other, with ABS winning
on *controllability* and the locked slide winning on *distance*.

## 6.2 Sensitivity and limitations

The weakest modelling choices, ranked by how likely each is to flip the
conclusion rather than just nudge a number:

1. **Planar, no pitch dynamics.** N_f is the quasi-steady load-transfer
   formula; the bike cannot rotate about the front contact. §6.3 shows this
   is not cosmetic — it hides a complete mode of failure.
2. **Single-μ Dugoff tire, no μ(v) rolloff.** The locked-wheel penalty is
   under-counted. A realistic μ_slide / μ_peak ≈ 0.7 would widen the
   cadence/ABS gap over the locked baseline by roughly the same factor.
3. **Hall + MA-4 estimator lag (~20 ms).** Already acknowledged in
   `ASSUMPTIONS.md` → `ABSController` §5 — the reason MVP ABS can't meet
   PLAN's peak-|λ|<0.30 oracle, and the single biggest lever on ABS
   performance in this model.
4. **No rider-in-the-loop.** F_hand / V_pwm is a prescribed ramp-and-hold,
   not a reaction-delayed squeeze on actual feedback. Comparing against a
   human who instantly applies peak force is generous; a realistic 0.5–1.0 s
   reaction time shifts all three stop distances up by v₀·t_react ≈ 4–8 m —
   a larger effect than the controller-choice differences above.
5. **No front-fork compliance, cargo sloshing, or tyre sidewall dynamics.**
   Each adds tens of milliseconds of effective actuator lag the FSM doesn't
   model.

## 6.3 Pitchover check

All three controllers peak at **|a_x| ≈ 8.83 m/s² ≈ μ_peak · g** — the tyre
saturates briefly on every strategy. The cargo-bike pitchover limit with the
default geometry is

```
a_pitch = g · ℓ_f / h = 9.81 · (1.2 − 0.7) / 1.0 = 4.91 m/s²
```

(ℓ_f = L − a = 0.5 m front-contact→CG, h = 1.0 m CG height; see
`configs/default.toml`). **Every strategy crosses the pitchover limit by
roughly 1.8×.** On a real loaded cargo e-bike the rear wheel lifts and the
rider goes over the bars before the front tyre reaches μ_peak — the planar
model is answering the wrong question, treating μ as the binding constraint
when h/ℓ_f actually is.

The practical implication is sharper than "add a caveat": for this vehicle
class on high-μ surfaces the controller's job is not to extract maximum μ
from the tyre — it is to **cap a_x at ≈ 4.9 m/s² to keep the rear wheel on
the ground.** None of the three controllers modelled here does that, because
none of them knows about pitchover. A useful next extension (beyond the
low-μ scenario in §6.1) is either a closed-loop a_x-limiting outer loop on
top of the ABS FSM, or a 2-DOF pitch plant that lets N_r → 0 and fails the
sim honestly when it does. Either change would reframe the comparison from
"shortest stop on dry asphalt" — which is pitchover-limited, not
tyre-limited — to "preserved steering at fixed deceleration *below*
pitchover on a low- or mixed-μ surface," which is the scenario this vehicle
class actually buys an ABS for.

## 7. Conclusion

On a 30 km/h dry-asphalt panic stop the ABS FSM cuts time-locked from 72 %
(human baseline) to 12 % and stops in 7.22 m, edging cadence's 7.81 m on
controllability while ceding ≈ 2.5 m to the locked human slide that simply
surrenders steering for distance. The headline finding is therefore a
*controllability* win, not a distance win — the expected null result on a
high-μ surface where the Dugoff tyre's sliding μ equals its peak μ. The most
consequential limitation is that the planar plant has no pitch degree of
freedom: every controller crosses the 4.91 m/s² pitchover limit by ≈ 1.8×,
so on a real loaded cargo e-bike the binding constraint is the rear wheel
lifting, not the tyre saturating. The natural next extension is a low-μ
scenario (μ_peak ≈ 0.35, wet asphalt or paint) where sliding μ drops well
below peak and the locked-slide baseline collapses — the operating point
this vehicle class actually buys an ABS for.
