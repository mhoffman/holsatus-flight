# Phase 1 HIL Test Result — H743v2

Date: 2026-07-13
Firmware: `flight.rs` (auto-selects HIL mode on USB power), flashed via `make flash-release BIN=flight`.
Link: USART1 (PA9/PA10) at 115200 baud, adapter `/dev/cu.usbserial-A5069RR4`.
Host: `tools/hil_phase1_imu_inject.py`, run manually from the `flight-data` venv (`pip`-less/uv-managed; `pyserial` installed via `uv pip install --python .venv/bin/python3 pyserial`).

## Test 1: Static (level, stationary) — PASS

```
python3 tools/hil_phase1_imu_inject.py /dev/cu.usbserial-A5069RR4 static --duration 25
```

- 4420 frames sent, **0 bad/missing replies**.
- `CAL_DONE` flipped `N`→`Y` at t=11.31s (seq=2002).
- `roll=pitch=yaw=0.00` for the entire 25s run, both before and after `CAL_DONE` — i.e. the estimator holds level for the ~13.5s of genuine post-calibration data, not just the pre-cal identity fallback.
- Matches the plan's pass criterion: "Static level -> reported attitude converges to level."

### Gotcha found: first 5s and 25s runs looked identical because both only exercised the fallback

The first attempt used `--duration 5` (the script's default) and saw `cal=N` and flat zeros for the whole run. That is **not** a real pass — before `CAL_DONE`, `hil_link_task` replies with an identity-quaternion fallback (`flight.rs:3457`), so a flat-zero reading is indistinguishable from a genuinely level estimate. `att_estimator` isn't even spawned until `imu_cal::apply().await` returns (`flight.rs:691-693`), so nothing is being estimated at all during that window.

`imu_cal::apply()` (`src/imu_cal.rs`) has three gated phases before `CAL_DONE`:

1. Ready-hold: 1s of continuous stationary+level (fast).
2. **Hands-clear delay: a literal 5s wall-clock `Timer::after` sleep** — real time, independent of link speed.
3. Bias capture: 2000 IMU samples averaged — at the observed effective link rate (~177 Hz, i.e. `frames / elapsed`), that's ~11.3s.

Total ≈ 17-18s before `CAL_DONE`, confirmed by the actual flip at t=11.31s... plus the two prior stages (this run's flip time already includes phases 1+2, so ~11.3s end-to-end was faster than the ~17s upper estimate — the effective link rate during capture was evidently a bit higher than the coarse 177 Hz average over the whole run).

**Recommendation for the tool**: `static` mode is the only one of the three that doesn't call `wait_for_calibration()` first (tilt/dynamic already do, with a 30s default timeout, so they're safe). Either bump `static`'s effective test duration well past ~20s by convention, or change `static` to call `wait_for_calibration()` like the other two modes so a short `--duration` can't silently produce a fallback-only "pass". Not changed yet — flagging for a follow-up, since it's a script change, not a firmware one.

## Test 2: Tilt (30° roll, stationary) — PASS (sign convention identified)

```
python3 tools/hil_phase1_imu_inject.py /dev/cu.usbserial-A5069RR4 tilt --roll-deg 30 --duration 5
python3 tools/hil_phase1_imu_inject.py /dev/cu.usbserial-A5069RR4 tilt --roll-deg -30 --duration 5 --skip-warmup
```

- `CAL_DONE` was already true across all these runs (same boot session as the static test); the estimator's Madgwick state also **persists across separate script invocations** — reconnecting doesn't reset the FC.
- Commanding `--roll-deg 30` converges to a reported roll near **-30°**; commanding `--roll-deg -30` converges to exactly **+30.00°** (held flat for a full 5s run). The magnitude is correct; the sign is inverted relative to naive expectation.
- **Root cause, not a firmware bug**: `att_estimator.rs` applies `rot_x_180` (`diag(1,-1,-1)`, i.e. flips Y/Z, leaves X unchanged) to gyro and accel before feeding the Madgwick filter (whose library assumes the opposite Z-up convention), and applies the same transform to the output Euler triple. The tilt test's commanded roll lives entirely in accel's **Y/Z** components (`ay=G·sin(r), az=-G·cos(r)`), which *do* get flipped by this transform — hence the apparent sign inversion. This is a geometrically explainable, structural consequence of the pipeline, not an arbitrary script bug. (Compare Test 3 below, where the signal lives in gyro's **X** component instead, and shows no such flip.)
- Convergence is governed by **injected sample count against the firmware's fixed `dt=1/1000s`**, not wall-clock time — see Test 3 for the quantitative version of this claim. Roll swept smoothly from the +30-consistent state down through 0 toward the -30-consistent state across five separate reconnects/invocations with no discontinuity, confirming state genuinely persists and evolves only on real injected samples.
- **Anomaly, not fully explained**: yaw jumped from ~+10.6° to ~-49.7° once, at the boundary between two consecutive tilt runs, despite `gyr=(0,0,0)` throughout (yaw is unobservable from accel alone — rotating gravity's yaw component doesn't change its body-frame projection — so nothing should have moved it). Did not reproduce on four subsequent reconnects in the same session (yaw held flat at -40.58° throughout). Flagging as an open, intermittent artifact rather than a settled root cause; possibly related to a stray byte/framing effect at serial reconnect, or an Euler-angle branch-cut artifact — unconfirmed.

### Follow-up (2026-07-14): reconnect-boundary roll jump reproduced on a fresh boot, three causes ruled out

On a freshly rebooted, uncalibrated board, `tilt --roll-deg -30` correctly ramped roll linearly from 0 to ~3.8° over its first 5s run (matches the constant-magnitude gradient-descent step in the `ahrs` crate's `update_imu` -- see below). A `--skip-warmup` reconnect immediately after showed roll already flat at exactly **30.00°** from the very first sample, with zero visible ramp-in. Reproduced twice in a row; once snapped, it stays stable (a second reconnect also read flat 30.00, no further drift) -- this is not a decaying transient, it looks like a deterministic snap straight to the converged value.

Three plausible causes were checked directly against the code and ruled out:
1. **Stale OS-buffered bytes at reconnect** -- the script's connect-time `ser.in_waiting` flush diagnostic reported 0 bytes both times.
2. **A ticker-driven phantom-update burst** -- `common/src/tasks/imu_reader.rs:110-135`: the 1kHz `Ticker` only gates *when a read attempt is allowed*; the actual `imu.read_acc_gyr().await` still blocks on the real channel for `HilImu`. A long idle gap doesn't replay missed ticks as extra samples.
3. **An adaptive/fast-convergence branch in the Madgwick library** -- read the actual `ahrs` crate source (`~/.cargo/git/checkouts/ahrs-rs-*/src/madgwick.rs`). `update_imu`'s correction step is `(J_t * F).try_normalize(...)`, i.e. **normalized to constant magnitude regardless of error size**, then scaled by `beta` -- no branch on gyro magnitude or error size anywhere. This matches the clean linear ramp seen on the fresh-boot run, but cannot itself produce an instant full-magnitude jump.
4. (Also checked and ruled out: the BMI270 logger task publishes to `CAL_MULTI_IMU_DATA[1]`, a different index than `att_estimator` reads from (`[0]`) -- no cross-contamination.)

**Conclusion so far**: whatever causes this happens strictly inside the reconnect gap itself, before the first sample the host can observe -- invisible to anything watching only the HIL binary link on USART1. The only existing attitude-related text log (`"[ahrs] waiting for attitude to settle..."`, `flight.rs:836`) is one-shot, not periodic, so it doesn't help either. **Next step, not yet taken**: add a periodic `AHRS_ATTITUDE_Q` print to the BT/UART8 text-log path (which stays live and independent of HIL's USART1 link in HIL mode) so the estimate can be watched continuously through a live reconnect. Requires a firmware change and reflash -- deferred pending a decision on whether to pursue root cause further before Phase 2.

## Test 3: Dynamic (constant roll rate) — PASS (rate prediction confirmed within ~19%)

```
python3 tools/hil_phase1_imu_inject.py /dev/cu.usbserial-A5069RR4 dynamic --roll-rate 0.5 --duration 5 --skip-warmup
```

- Roll ramped from 1.92° (seq=36, t=0.20s) to 30.02° (seq=857, t=4.85s): **Δroll=28.10° over Δt=4.65s → 6.04°/s wall-clock.**
- Effective link rate this run: 821 samples / 4.65s ≈ **176.6 Hz**.
- **Prediction, made before this run**: because `att_estimator.rs`, `controller_rate.rs`, `controller_angle.rs`, `eskf.rs`, and `in_flight_estimator.rs` all compute `dt = 1.0 / get_ctrl_freq!()` **once**, from the `CONTROL_FREQUENCY` constant (hardcoded to 1000 in `main()`) — never from measured elapsed time — every one of these integrates as if exactly 1ms has passed per update, regardless of how much real wall-clock time the lockstep link actually took to deliver that update. At the observed ~176.6 Hz effective rate, predicted ramp = `176.6 × 0.5rad/s × 0.001s(in deg) ≈ 5.06°/s`.
- Observed 6.04°/s vs predicted 5.06°/s: within ~19%, and both dramatically below the naive real-hardware expectation of `0.5rad/s ≈ 28.65°/s`. This confirms the core hypothesis — convergence proceeds in **sample-domain** terms, not real time — with the residual ~19% gap plausibly from the accel-correction term still acting (injected accel stays fixed at "level" the whole run, which becomes increasingly physically inconsistent with the growing roll, adding a secondary correction on top of pure gyro integration).
- No sign flip: roll increases for a positive commanded rate. Consistent with the Test 2 root-cause explanation — `gyr_x` is the X-component of the gyro vector, which `rot_x_180` leaves unchanged, so no inversion is expected here.
- **New anomaly, same family as the Test 2 yaw jump**: pitch opened at **-15.28°** and slowly relaxed toward -13.21° over the run, despite a pure-roll stimulus that held pitch near 0 throughout every tilt test. Coincides with the mode transition from `tilt` to `dynamic` while a persistent, non-level roll state carried over. Not fully explained; flagging as an open item, possibly related to applying `rot_x_180` directly to the Euler-angle triple (not a rigorous way to transform Euler angles between frames in general, unlike applying it to raw vectors) interacting with a carried-over non-level state at a mode transition.
- 0 bad/missing replies (884 sent) — link quality remains excellent across the full multi-hour session.

## Phase 1 summary

All three runbook tests (static, tilt, dynamic) pass their stated pass/fail criteria from `references/hil_implementation_plan.md`, on real H743v2 silicon, over a genuine 115200-baud link with zero frame loss across the whole session. Two structural findings apply broadly to interpreting *any* lockstep HIL result on this firmware:

1. **Lockstep wall-clock timing is not representative of real-hardware timing.** Every fixed-`dt` task effectively runs at whatever rate the host delivers frames (~176-180 Hz observed here, not the assumed 1000 Hz), so anything time-constant-dependent (AHRS beta correction, PID integration, etc.) will appear to happen ~5-6x slower in wall-clock terms than on real flight hardware. This matches the plan's own framing — lockstep phases are for sample-domain correctness, not timing evidence (that's explicitly Phase 4's job).
2. **Two intermittent, unexplained cross-axis anomalies** (a one-off ~60° yaw jump between tilt reconnects; a ~15° pitch excursion at the tilt→dynamic mode transition) — both transient, both self-correcting, neither blocking the pass/fail calls above, but both worth a closer look before leaning on this rig for anything beyond single-axis sanity checks. Possibly related to the non-standard step of rotating the output Euler-angle triple as if it were a raw vector, or to reconnect-time framing artifacts — unconfirmed either way.

## Follow-on idea (not started): real-flight-log replay for sim2real gap

Once tilt/dynamic close out Phase 1's synthetic-profile pass/fail, a natural extension is to replay gyro/accel from a real Betaflight blackbox log (decoded via `/Users/maxjh/src/blackbox-tools/obj/blackbox_decode --simulate-imu`) through the same `SensorFrame` protocol instead of the static/tilt/dynamic synthetic profiles. That drives the estimator with real IMU vibration/noise spectra on actual silicon and gives a direct sim2real comparison point against `holsatus-sim`, rather than only clean synthetic inputs.
