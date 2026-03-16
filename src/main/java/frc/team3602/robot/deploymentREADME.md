# Deployment README

## Purpose

This file is a practical deployment and test checklist for the robot.

Use it after:

- code changes
- wiring changes
- mechanism repairs
- sensor adjustments
- drivetrain or turret tuning changes

The goal is to catch problems in a safe order before the robot is asked to run
full teleop or autonomous routines.

## Before You Deploy

- Verify the battery is charged and firmly connected.
- Verify the robot is on blocks if any mechanism may move unexpectedly.
- Verify the turret is physically placed in the team's expected start position
  of `90` degrees.
- Verify the team is still using the current public turret direction map:
  forward = `0`, left = `90`, right = `-90`, rear = `180`.
- Remember that the software still keeps a private `0` through `360` internal
  travel model so it can handle the rear seam correctly.
- Verify fuel is removed from the robot for the first motion checks.
- Verify the turret lead-math assumptions still match the current shooter setup:
  `70` degree launch angle and `20` inch shooter height.
- Verify Driver Station and radio are connected normally.
- Verify the correct branch and commit are deployed.

## After Deploy

- Enable the robot while still on blocks.
- Watch Driver Station for CAN errors, brownout warnings, or spammed faults.
- Confirm SmartDashboard values are updating.
- Confirm no subsystem moves unexpectedly at enable.

## Control Map To Verify

### Driver Controller 0

Drive:

- Left stick = translation
- Right stick X = rotation
- Right trigger = turbo

Intake / Pivot:

- Left bumper = intake on while held
- Right bumper = press once to send intake down

Climber:

- POV up / down = climber, only if enabled

### Operator Controller 1

Turret:

- Right trigger = track alliance tower
- POV up / down / left / right = turret presets

Shooting:

- `B` = combined tracked lerp shot
- `A` = fixed `-41.5` shooter preset
- `X` = fixed `-44.0` shooter preset
- `Y` = manual spindexer / transfer feed
- Left trigger = failsafe shot
- Failsafe shot = turret `180.0` degrees (rear) and shooter `-62.5` RPS

Intake / Pivot Support:

- Left bumper = stop intake and raise pivot

## Phase 1: Safe Idle Checks

- Disable and re-enable once to confirm the robot comes up cleanly.
- Verify the turret holds position when idle.
- Verify the pivot holds position when idle.
- Verify the drivetrain does not creep when sticks are centered.

## Phase 2: Drivetrain Check

- Drive slowly in open space.
- Confirm forward, strafe, and rotate all behave normally.
- Hold turbo and confirm the robot speeds up.
- Release turbo and confirm normal speed returns.
- Watch the dashboard pose values and confirm they move smoothly.

## Phase 3: Turret Check

- Press operator POV up and confirm the turret goes to the rear preset.
- Press operator POV down and confirm the turret goes to the right-side preset.
- Press operator POV left and confirm the turret goes to the left corner preset.
- Press operator POV right and confirm the turret goes to the right corner preset.
- If the turret moves near the rear seam, confirm it goes around the `0` / `360`
  boundary instead of trying to wrap across it.
- Hold operator right trigger and confirm the turret actively tracks the alliance
  tower.
- Release operator right trigger and confirm the turret stays stable.
- While doing these checks, watch:
  `Turret/MeasuredAimDegrees`, `Turret/RequestedAimDegrees`,
  `Turret/AimErrorDegrees`, `Turret/MeasuredTravelDegrees`,
  `Turret/RequestedTravelDegrees`, and `Turret/MotorVoltage`.

## Phase 4: Shooter Check

- Press and hold operator `B`.
- Confirm the turret tracks the current alliance tower.
- Confirm the shooter follows the current interpolated lerp speed.
- Confirm the spindexer and transfer stay off at first, then begin feeding only
  after the shooter and turret are ready.
- Once the shot becomes ready, confirm feed stays on smoothly instead of
  chattering on and off during the shot.
- Confirm driver controller 0 begins rumbling when the shot becomes ready.
- Release operator `B` and confirm the shooter and feed stop cleanly.
- Confirm the rumble stops when operator `B` is released.
- Press operator `A` and confirm both shooter motors spin up together.
- Release operator `A` and confirm both shooter motors stop.
- Press operator `A` again and confirm both shooter motors spin up together a second time.
- Repeat the same restart test with operator `X`.

This step is important because the shooter follower issue was recently fixed.
We want to verify the second shooter motor still follows correctly after a stop.

## Phase 5: Spindexer And Transfer Check

- With no fuel loaded, hold operator `Y`.
- Confirm the spindexer and transfer/X44 both run together smoothly.
- Release operator `Y` and confirm both stop immediately.
- Listen for any obvious speed mismatch, chatter, or slipping.

## Phase 6: Shooter Vibration Check

- Send the turret to a fixed preset.
- Run shooter `A`, then `X`.
- Watch whether the turret stays put while the shooter is running.

This step verifies the recent turret brake-mode change.

## Phase 7: Manual Shot Check

- Load one fuel piece.
- Aim the robot at a safe scoring direction.
- Hold operator `B` for the combined tracked lerp shot.

Watch for:

- clean fuel handoff from spindexer to transfer
- no hesitation entering the shooter
- no double-feed
- turret staying aimed during the shot
- both shooter motors sounding synchronized

While operator `B` is still held, also verify the intake and pivot are not
blocked:

- Press driver left bumper and confirm the intake can still turn on and off.
- Press driver right bumper and confirm the intake can still drop.
- Press operator left bumper and confirm the intake can still stop and raise.

Also verify the protected combined-shot controls:

 - While operator `B` is held, press operator `A`, `X`, and POV presets one at a time.
- Confirm those overlapping operator shooter / turret controls do not steal the
  shot away while `B` is still held.
- Release operator `B` and confirm the normal operator shooter / turret controls
  work again immediately.

Then also verify the older split workflow still works:

- Hold operator right trigger so the turret tracks the alliance tower.
- Start the shooter with the intended preset.
- Wait for spin-up.
- Hold operator `Y` briefly to feed.

## Phase 8: Failsafe Shot Check

- Press and hold operator left trigger.
- Confirm the turret moves to `180.0` degrees, which is the rear direction in
  the public signed turret convention.
- Confirm the shooter spins up to the fixed failsafe target of `-62.5` RPS.
- Confirm feeding starts only after the built-in delay.
- Confirm the routine stops the shooter and feed on its own after the fixed feed window.
- Release the trigger early during one test run and confirm shooter and feed stop immediately.

## Phase 9: Autonomous Command Check Without Fuel

- Run the selected autonomous with no fuel loaded first.
- Confirm the path starts correctly.
- Confirm `autonPrepareTowerShot` runs after the drive segment.
- Confirm `autonFireShot` runs for its expected window.
- Confirm `autonStopShooter` stops the shooter and spindexer.
- Confirm `autonStowTurret` returns the turret to its start angle.

## Phase 10: Autonomous Shot Check With Fuel

- Load one fuel piece.
- Re-run the simplest autonomous.
- Watch whether the shooter is ready before feed begins.
- Watch whether the turret stays aimed through the shot.
- Watch whether the fuel path stays smooth from spindexer to transfer to shooter.

## Dashboard Values To Watch

- `Robot X`
- `Robot Y`
- `Robot Angle`
- `Pose Distance`
- `Shooter Lerp Speed`
- `Pigeon Angle`
- `Turret/MeasuredAimDegrees`
- `Turret/RequestedAimDegrees`
- `Turret/AimErrorDegrees`
- `Turret/MeasuredTravelDegrees`
- `Turret/RequestedTravelDegrees`
- `Turret/MotorVoltage`

These values help separate aiming problems from feed problems.

## If Something Looks Wrong

- Stop and test with no fuel first.
- If only one shooter motor seems active, re-check shooter follower behavior.
- If the turret drifts while shooting, re-check turret brake mode and turret hold behavior.
- If the turret takes a shortcut across the rear seam, re-check the new
  `0` through `360` travel model and startup-angle assumptions.
- If shots consistently arc high or low, re-check the turret lead-math
  assumptions for shooter height and launch angle.
- If fuel hesitates or bounces, focus on the spindexer-to-transfer handoff first.
- If autonomous feeds too early, remember the current beta auto still allows a timeout-based shot.

## Sign-Off

Before the robot is considered ready, confirm:

- drivetrain works
- turret holds and aims
- both shooter motors spin reliably after repeated stop/start cycles
- spindexer and transfer run together cleanly
- one manual shot works
- one simple autonomous works
