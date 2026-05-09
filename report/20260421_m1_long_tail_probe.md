# 2026-04-21 M1 long-tail probe

## Goal
- Shorten the post-start tail
- Remove the early left/up detour in the carrier path
- Keep `prehold` start reliable

## Code changes tested
- `src/easydocking_control/src/docking_controller.cpp`
  - strengthened far-field `APPROACH` shaping toward the global intercept corridor
  - blended the approach command frame toward the intercept route direction
  - increased far-field along-speed / pursuit aggressiveness
- `scripts/wait_for_docking_window.py`
  - added a guard that blocks local prediction release when the global intercept geometry is clearly bad (`int_local_veto`)
  - tested a stricter primary-gate min-after-orbit delay; reverted because it caused `start-window-fail`

## Smoke runs

### `20260421_162319_px4_sih`
- `classification=geometry-fail`
- `time_to_first_tracking_sec=15.200`
- `post_start_path_length_m=154.834`
- `docking_path_length_m=26.401`
- better than the `~180–200m` family, but still not the desired short/tangent-shaped intercept

### `20260421_162904_px4_sih`
- `classification=geometry-fail`
- `time_to_first_tracking_sec=15.200`
- `post_start_path_length_m=180.158`
- `docking_path_length_m=59.215`

### `20260421_163333_px4_sih`
- `classification=geometry-fail`
- `time_to_first_tracking_sec=14.500`
- `post_start_path_length_m=185.393`
- `docking_path_length_m=65.807`

### `20260421_164041_px4_sih`
- `classification=geometry-fail`
- `time_to_first_tracking_sec=14.700`
- `post_start_path_length_m=185.710`
- `docking_path_length_m=55.616`

## Main finding
- The current bottleneck is still **start geometry / intercept timing**, not just controller aggressiveness.
- In the bad family, the carrier is still released into a geometry that produces the visible early left/up loop before it settles onto the lower tangent corridor.
- Purely making `APPROACH` more aggressive does not reliably remove that loop.
- Hard-delaying the primary global-intercept release regressed start reliability and was reverted.

## Current conclusion
- Keep the `int_local_veto` safeguard.
- The next useful step is to refine the **global intercept candidate / release timing policy** so release happens only when the selected corridor is already the lower-energy tangent family, without reintroducing `start-window-fail`.

## Update: 2026-04-22

### Change
- `scripts/wait_for_docking_window.py`
  - the global intercept selector no longer assumes the carrier reaches the ideal corridor point
  - it now predicts the **actually reachable** carrier position at candidate time `tau`
  - `prediction_along_m` / `prediction_lateral_m` are evaluated in the future mini tangent frame from that reachable point
  - the global candidate score now includes this real tangent-frame geometry term

### Result

#### `20260422_001520_px4_sih`
- `classification=geometry-fail`
- `time_to_first_tracking_sec=5.100`
- `post_start_path_length_m=111.513`
- `docking_path_length_m=59.599`
- `start_intercept_pred_along_m=-43.756`
- `start_intercept_pred_lat_m=1.292`
- path shape matches the intended lower tangent family much better

#### `20260422_001807_px4_sih`
- `classification=geometry-fail`
- `time_to_first_tracking_sec=5.400`
- `post_start_path_length_m=106.410`
- `docking_path_length_m=67.193`
- `start_intercept_pred_along_m=-44.320`
- `start_intercept_pred_lat_m=1.656`
- again enters the short-path family instead of the old `~180m` tail family

### New conclusion
- This change appears to fix the main release-family problem:
  - `time_to_first_tracking_sec` dropped from about `14.5–15.2s` to about `5.1–5.4s`
  - `post_start_path_length_m` dropped from about `155–186m` to about `106–112m`
- The main bottleneck has shifted again:
  - release geometry is much better
  - terminal / docking precision is still the reason these runs remain `geometry-fail`

### 5-run mini-batch check
- runs:
  - `20260422_001520_px4_sih`
  - `20260422_001807_px4_sih`
  - `20260422_002154_px4_sih`
  - `20260422_002416_px4_sih`
  - `20260422_002638_px4_sih`
- aggregate:
  - `time_to_first_tracking_sec`: min `5.100`, max `5.400`, avg `5.184`
  - `post_start_path_length_m`: min `104.046`, max `111.513`, avg `108.153`
  - `docking_path_length_m`: min `59.599`, max `67.193`, avg `62.538`
  - `start_intercept_pred_along_m`: avg `-44.472`
  - `start_intercept_pred_lat_m`: avg `1.508`
  - `start_intercept_pred_score`: avg `0.521`
- classification mix:
  - `1/5` = `completed-but-not-final`
  - `4/5` = `geometry-fail`

### Batch conclusion
- The short-path / correct-family release now looks consistent across this 5-run probe.
- The remaining failures are no longer dominated by the old long-tail release problem.
- The next work item should move to **terminal geometry / docking precision**, while keeping this new release-family behavior as the baseline.
