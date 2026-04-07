# Tangent Release Goal And Validation

## Final task goal

This project does not aim to merely make the relative distance smaller.
The intended task is an aerial-carrier style heterogeneous cooperative docking problem:

- The `mini` vehicle behaves like a fixed-wing receiver.
- The `carrier` behaves like the active chaser and capture platform.
- Before release, `mini` stays on a fixed-altitude waiting orbit.
- When a true catch window exists, `mini` leaves the orbit along the local orbit tangent.
- During that tangent segment, `mini` is passive and should not immediately curl inward toward the deck.
- `carrier` must actively chase, align, and create the final docking geometry.
- Only after the geometry becomes favorable should the system hand off from tangent flight to terminal docking guidance.

In short:

- `orbit` creates a predictable target motion.
- `tangent release` creates a physically plausible fixed-wing exit.
- `carrier chase` creates the capture opportunity.
- `terminal docking` closes the final relative error.

## State semantics

### SEARCH

- `mini` remains on its waiting orbit.
- `carrier` has not committed to aggressive close-in interception.
- The objective is to create a feasible catch window, not to force immediate contact.

### APPROACH

- `carrier` begins chasing the orbiting `mini`.
- `mini` should still be orbiting unless a release gate explicitly accepts tangent release.
- No inward spiral release is allowed in this state.

### TRACKING

- `carrier` is close enough that a future tangent handoff may become feasible.
- If `mini` is released, it should follow the tangent as a passive straight segment.
- `carrier` remains the dominant active platform.

### DOCKING

- This state should start only after the tangent segment has created a usable corridor.
- `mini` may then transition from passive tangent flight to terminal cooperative behavior.
- Only here is it acceptable for `mini` to slow down and more directly cooperate with final capture.

### COMPLETED

- Relative position error is within the docking tolerance.
- Relative speed is sufficiently matched.
- After this point the pair can be treated as one coupled system dominated by `carrier` dynamics.

## Required behavioral constraints

The following constraints are not optional. A run that violates them is not considered semantically correct even if `min_distance` is small.

1. `mini` must not leave orbit by immediately turning inward toward the deck.
2. After release, the first segment must be locally tangent to the waiting orbit.
3. `carrier` must do the majority of the chase work before terminal capture.
4. `mini` must not enter early terminal cooperation while the catch geometry is still poor.
5. A tangent segment that is armed and then canceled almost immediately is not considered a valid tangent release.

## What must be logged

Every candidate run should preserve enough data to answer these questions:

- When was tangent release armed?
- How long did the tangent segment remain active?
- Why was tangent exit cleared?
- What was the orbit radius at release, and how much did it shrink in the first second?
- What was the carrier-to-mini geometry in the tangent frame:
  - forward progress
  - lateral error
  - relative closing along tangent
  - predicted time to align with the tangent corridor
- At handoff:
  - distance to deck
  - lateral error to tangent corridor
  - relative speed
  - current docking phase

## Temporary algorithm validation requirements

These are the working requirements for the current PX4 migration stage.

### Release semantics

- `mini` must remain on orbit until an explicit release gate accepts.
- A release is only valid if it leads to a visible tangent segment.
- The tangent segment must last long enough to be meaningful, not a one-frame artifact.

### Tangent segment quality

- In the first `1.2 s` after release, the orbit radius must not collapse inward by more than a small tolerance.
- A practical working tolerance is:
  - `radius_drop_1p2s <= 0.5 m`
- If the radius rapidly decreases, the release is classified as an inward spiral, not a tangent exit.

### Handoff semantics

- Exiting tangent flight should be driven by geometry, not by a decorative timer alone.
- A valid handoff should require all of:
  - carrier close enough to the tangent corridor
  - lateral error small enough
  - forward progress favorable enough
  - docking phase sufficiently mature

### Terminal cooperation

- Before handoff, `mini` is passive.
- After handoff and especially in `DOCKING`, `mini` may reduce speed and cooperate with final alignment.
- Terminal slowdown is not allowed to begin so early that it destroys the tangent-release semantics.

## Practical pass/fail criteria for current iteration

### A run passes semantic tangent validation if all of the following are true

- A tangent release is armed.
- Tangent exit remains active for a nontrivial duration.
- The first release segment does not show inward radius collapse.
- `mini` does not immediately curl toward the deck.
- `carrier` later reduces the tangent-frame lateral error and creates a plausible handoff.

### A run fails semantic tangent validation if any of the following happen

- No tangent release is armed.
- Tangent exit is cleared almost immediately with no meaningful straight segment.
- The orbit radius shrinks quickly right after release.
- `mini` transitions into inward spiral behavior instead of tangent flight.
- `mini` flies straight forever because release happened when no real catch window existed.
- `carrier` never creates a usable tangent-frame handoff geometry.

## Current engineering takeaway

Two different failures must be distinguished:

1. `inward spiral failure`
   - release happens, but `mini` curls inward too early
2. `straight fly-away failure`
   - tangent release is preserved, but release timing is too early and `carrier` cannot catch

The target solution is neither of those.
The target solution is:

- late enough release to create a real catch window
- long enough tangent segment to preserve fixed-wing semantics
- explicit geometry-based handoff into terminal docking
