# Aim lab (phone mouse vs real mouse, with auto-tuning)

```
python PC-part/game/aimlab.py --input mouse      # baseline with your normal mouse (do this once)
python PC-part/game/aimlab.py --input phone      # phone face-down, USB, tap "Mouse mode" on the phone
```
SPACE starts each level, ESC quits. Six levels (~4 min): Calibrate, Targets, Rhythm, Trace, Precision, Scroll
(phone: twist to scroll). Results show next to your latest mouse baseline.

**Auto-improve:** after a phone session `tune.py` updates `phone_params.json` (used by the next phone session and
by `PC-part/receiver/mvp_live.py`):
- `mount_yaw_deg`, `aspect`: from the first, uncorrected part of each aiming move vs the target direction,
  minus your own bias on the mouse. Damped (70 % per session) so it converges instead of oscillating.
- `tap_thr_min`: between your weakest rhythm taps and the strongest non-tap jolts in Trace.
- `scroll_omega`: above the fastest accidental twist outside the Scroll level.
- `still_flow_px`: from Precision jitter / timeouts.
- Reports extra lag vs the mouse (Trace).

Sessions: `PC-part/game/sessions/` (gitignored). Re-run the tuner by hand: `python PC-part/game/tune.py [session]`.

**Self-test (no phone):** `python PC-part/game/aimlab.py --input bot --headless --no-tune` (baseline), then
`--input bot --bot-rot 15 --bot-aspect 0.8 --headless` a few times. Measured: rotation error 15° → 3.9° → 1.0° → 0.2°,
vertical gain 0.80 → 0.96 → 0.98 → 1.00.
