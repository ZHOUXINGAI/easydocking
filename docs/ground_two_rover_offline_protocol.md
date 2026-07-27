# Ground Two-Rover Offline Planner Contract

Status: pure-software baseline. This code does not open ROS, MAVLink, serial,
LR24, PX4, Arduino, actuators, or vehicle executors.

## Geometry

All positions are in one surveyed ENU field frame: `x=East`, `y=North`, and
yaw/phase are counter-clockwise from East.

For an orbit center `O`, radius `R`, and external Carrier point `C`, let
`p=C-O`, `d=|p|`. The two exact radius vectors to the tangent points are:

```text
q = (R^2/d^2) p +/- (R sqrt(d^2-R^2)/d^2) perp(p)
T = O + q
```

This guarantees `|T-O|=R` and `(C-T) dot (T-O)=0`. The equivalent angular
offset is `acos(R/d)`. The historical `asin(R/d)` expression was not a general
external-point tangent and is intentionally covered by a regression test.

For `O=(0,0)`, `R=4.5`, `C=(-7,-6)`, and CCW Mini motion, the selected branch
is:

```text
T = (0.8883757492037598, -4.411438374071053)
tangent = (0.9803196386824563, 0.1974168331563911)
```

Because this true tangent is constructed from `C`, the segment `C->T` already
has the required terminal heading. Without a Carrier start-heading constraint,
the approach is a zero-curvature straight tangent, represented as
`CarrierApproach.kind=straight_tangent`. A future nonzero-curvature path must
add a measured start heading and a Dubins/biarc/clothoid contract.

## Leader State

The Carrier-owned leader implements:

```text
HOLD
  -> WAIT_MINI_STABLE_ORBIT
  -> PLAN_VALIDATED
  -> CARRIER_ARC/MINI_ORBIT
  -> MINI_TANGENT_EXIT
  -> SHARED_TERMINAL
  -> COMPLETE_HOLD
```

Every runtime violation enters locally latched `ABORT_LATCHED`. Only the local
operator-facing `reset_to_hold()` can clear it. A new attempt must use a new,
positive attempt/plan ID.

Mini orbit qualification requires continuous radius, velocity-direction, and
health validity for at least one accumulated revolution. One immutable plan is
created per attempt. Completion requires the distance and terminal-line checks
to remain valid for `completion_hold_ms`; one control sample cannot complete
the attempt.

Signed terminal gap is:

```text
signed_gap = (Mini - Carrier) dot tangent_direction
```

Carrier-ahead therefore means `signed_gap <= carrier_ahead_tolerance_m`. This
guard is checked on the tangent-trigger sample before any terminal command is
published, and then on every sample through completion. Completion hold starts
only after both rovers have crossed the tangent point into the terminal
corridor.

## Runtime Transport Boundary

LR24 Pair B is only the compact runtime link between Carrier and Mini. It may
carry bounded Mini state, the immutable plan coordinates/identity, short
PlanCommand phase/speed primitives, HOLD, and Abort. It must not carry files,
repositories, source patches, build artifacts, bulk logs, or Codex/NATS
coordination traffic.

Code and file deployment uses GitHub/SSH outside the vehicle-control loop. NATS
is exclusively for Codex-to-Codex coordination and is never a vehicle runtime
transport. The offline leader remains protocol-neutral: a later Pair B adapter
serializes this contract but cannot change its geometry, timing, or safety
semantics.

## Fields Required By The Next Pair B Adapter

`VehicleState` / MiniState wire payload:

- `vehicle_id`, `sequence`, `sender_monotonic_ms`
- `frame_id`, `origin_id`
- `position`, `velocity`, `yaw_rad`, `health_ok`

`received_local_ms` is deliberately not transmitted. The receiving Carrier
adapter assigns it from its own monotonic clock when a valid packet arrives.
Non-finite coordinates, invalid identities, sequence regression, and sender
timestamp regression are rejected before planner use.

`GroundCorridorPlan`:

- `schema_version`, `plan_id`, `sequence`
- `frame_id`, `origin_id`
- `sender_monotonic_ms`, `valid_until_sender_monotonic_ms`, `validity_ms`
- `requested_validity_ms`, `required_validity_ms`, `validity_margin_ms`
- `post_tangent_reserve_ms`, `terminal_completion_budget_ms`,
  `completion_hold_ms`, `plan_timing_guard_ms`
- `validity_policy`, `validity_extended`
- `orbit_center`, `orbit_radius_m`, `turn_direction`
- `tangent_point`, `tangent_direction`, `tangent_phase_rad`
- `mini_phase_at_plan_rad`, `mini_exit_delta_rad`,
  `mini_arrival_delay_ms`, `required_stable_orbit_laps`
- `terminal_length_m`, `target_front_gap_m`
- `mini_speed_mps`, `mini_max_accel_mps2`, `carrier_max_speed_mps`,
  `carrier_max_accel_mps2`
- `mini_state_stale_ms`, `command_ttl_ms`, `local_command_watchdog_ms`
- `carrier_approach.kind/start/end/length_m/duration_ms/planned_speed_mps/curvature_per_m`

`GroundPlanCommand`:

- `schema_version`, `plan_id`, `sequence`, `target_role`, `phase`
- `sender_monotonic_ms`, `valid_until_sender_monotonic_ms`, `ttl_ms`
- `body_speed_mps`, `yaw_rate_radps`, `max_speed_mps`, `max_accel_mps2`
- `frame_id`, `origin_id`

The adapter must quantize and reject out-of-range values; it must not silently
change the immutable plan. Sender monotonic timestamps establish sender
ordering and packet TTL only. A receiver starts TTL at local receipt and never
compares the absolute monotonic clocks of Orin1 and Orin2.

Defaults are MiniState stale `300ms`, PlanCommand TTL `500ms`, local command
watchdog `750ms`, and requested plan validity `32000ms`. Plan schema version 2
uses a fail-closed `reject` policy. The planner computes:

```text
post_tangent_reserve =
    terminal_completion_budget
  + completion_hold
  + max(command_ttl, local_command_watchdog)
  + timing_guard

required_validity =
    ceil_to_100ms(mini_arrival_delay + post_tangent_reserve)
```

The defaults reserve `3350ms` after Mini reaches the tangent. If requested
validity is smaller than the computed requirement, plan creation aborts with
`plan_validity_insufficient`; validity is never silently extended.

`target_front_gap_m` is protocol metadata for the next closed-loop follower.
The current offline leader emits bounded speed/yaw-rate commands but does not
yet regulate that exact target gap. A passing replay must not be described as
implemented gap closure.

## Offline Usage

Run all deterministic replay scenarios:

```bash
python3 scripts/run_ground_docking_replay.py --scenario all
```

Write the same JSON result to a chosen offline path:

```bash
python3 scripts/run_ground_docking_replay.py \
  --scenario all \
  --output /tmp/ground_docking_replay.json
```

Run the focused tests:

```bash
python3 -m unittest discover -s tests -p 'test_ground_*.py' -v
```

These commands are offline only. Passing them does not authorize a
wheels-lifted or outdoor vehicle test.
