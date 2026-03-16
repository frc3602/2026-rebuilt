# Tuning README

## Purpose

This file is a practical tuning and commissioning guide for the robot.

It is meant to help students, mentors, and CSAs answer questions like:

- "Why did the turret move to the wrong place?"
- "What should we check before tuning PID?"
- "What is a software problem versus a mechanism problem?"
- "What order should we commission the robot after a repair?"

The goal is to tune and debug in a safe, readable order instead of changing many
things at once.

## Safe Tuning Order

When a mechanism behaves incorrectly, use this order:

1. Verify the robot is safe to test.
2. Verify the mechanism starts in the software's expected physical position.
3. Verify the command or setpoint being requested is the one you think it is.
4. Verify the sensor reading matches the real mechanism.
5. Verify motor direction and gearing assumptions.
6. Only then start changing PID or feedforward gains.

This order matters because many "bad PID" symptoms are really:

- wrong sensor zero
- wrong motor direction
- wrong setpoint convention
- mechanical slip
- stale or bad vision data

## Before Commissioning

- Put the robot on blocks if a mechanism might move unexpectedly.
- Remove game pieces for first tests.
- Confirm the battery is healthy.
- Confirm CAN devices appear normally in Phoenix Tuner.
- Confirm the code you think is deployed is actually the code on the robot.
- Confirm Driver Station shows the expected alliance and no major communication errors.

## Current Turret Angle Convention

The turret now uses the standard WPILib signed robot-relative convention for
public commands and geometry:

- `0` = forward
- `+90` = left
- `-90` = right
- `180` = rear

Important note:

- Internally, the turret still uses a private `0` through `360` travel model so
  the software can handle the rear seam correctly.
- Students and operators should think in the signed WPILib convention unless
  they are specifically debugging the seam logic.

## Turret Commissioning Checklist

Run these in order after turret repairs, code changes, or sensor resets.

1. Physically place the turret in the team's expected startup position.
   The current expected startup position is `90` degrees, which points left.
2. Power on and enable carefully.
3. Confirm the turret does not immediately jump to an unexpected angle.
4. Test fixed presets first:
   - rear preset
   - right preset
   - left corner preset
   - right corner preset
5. Test a simple fixed command before testing field tracking.
6. Test alliance-tower tracking only after fixed-angle commands look correct.
7. Test moving-shot lead behavior only after basic tracking looks correct.

## If The Turret Goes To The Wrong Position

Below is a symptom-based list of likely causes and good first fixes.

### 1. The turret is wrong by a large constant offset

Possible causes:

- The turret was not physically staged at the expected startup angle before boot.
- The Falcon rotor sensor was seeded from the wrong real-world position.
- The mechanism slipped on a shaft, pulley, belt, or coupling.
- The software angle convention is being interpreted incorrectly by the operator.

Good first fixes:

- Re-stage the turret physically to the expected startup angle before rebooting.
- Confirm everyone is using the current convention:
  `0` forward, `+90` left, `-90` right, `180` rear.
- Command a known fixed preset and compare the real turret to the expected direction.
- Inspect for loose pulleys, couplers, gears, set screws, or chain/belt slip.
- If the offset is repeatable every boot, re-check the startup seeding assumption
  before touching PID gains.

### 2. The turret moves the opposite direction from what the command implies

Possible causes:

- Motor inversion is wrong.
- Sensor sign is wrong.
- The mechanism's real positive direction does not match the software's assumed positive direction.

Good first fixes:

- Command a small known move and watch whether the reported angle increases or decreases.
- Confirm that asking for a more positive angle really moves left, not right.
- Verify the rotor-position math matches the real mechanism direction.
- Fix direction assumptions before tuning PID.

### 3. The turret reaches the correct general area but stops short

Possible causes:

- `kP` is too low.
- Static friction is larger than the controller can overcome.
- Current limiting is too aggressive.
- Brake friction or mechanism drag is too high.
- The turret is hitting a soft mechanical bind.

Good first fixes:

- Check for rubbing, cable drag, chain tension issues, or hard stops.
- Verify the turret can move freely by hand when safe to do so.
- Increase `kP` gradually only after confirming the mechanism is free.
- Consider adding or revisiting feedforward only after the basic direction and zero are correct.
- Check that current limits are not preventing the motor from producing enough torque.

### 4. The turret overshoots or oscillates around the target

Possible causes:

- `kP` is too high.
- `kD` may be needed or increased.
- There is mechanical backlash.
- The sensor reading is noisy or delayed.

Good first fixes:

- Lower `kP` slightly and retest.
- Add a small amount of damping only if the mechanism really is overshooting.
- Check for backlash in gear stages or couplers.
- Watch whether the sensor reading itself is stable while the turret is physically still.

### 5. The turret is correct on fixed presets but wrong when tracking the tower

Possible causes:

- The drivetrain pose estimate is wrong.
- The alliance value is wrong or missing.
- Vision corrections are poor or stale.
- The field target constant is wrong.
- The robot heading is wrong even if the turret hardware is fine.

Good first fixes:

- Verify fixed-angle commands first. If those are correct, the turret hardware is probably not the main issue.
- Check drivetrain pose on the dashboard.
- Check gyro heading against the real robot heading.
- Confirm the correct alliance is being reported.
- Check Limelight acceptance/rejection dashboard values.
- Test tracking with vision disabled versus enabled to see whether the pose estimator is the source of the error.

### 6. The turret seems correct until it gets near the rear of the robot

Possible causes:

- The issue is really rear-seam handling, not general position control.
- The requested angle is crossing from the `-180` side to the `180` side.
- A student is expecting shortest-path wrap behavior, but the code is intentionally avoiding that.

Good first fixes:

- Remember that rear is commanded as `180` in the public API.
- Watch whether the commanded target is moving through:
  `-170 -> 180 -> +170`
- Confirm the mechanism stays on one seam side briefly, then takes the long way around if needed.
- Do not "fix" this by forcing shortest-path math unless the team has also decided to change the physical travel model.

### 7. The turret is correct while standing still but wrong while driving

Possible causes:

- Drivetrain pose is drifting.
- The robot heading is wrong.
- Moving-shot lead assumptions are wrong.
- Shooter exit speed assumptions are wrong.

Good first fixes:

- Test fixed-angle presets while driving slowly to separate turret control from field tracking.
- Test tower tracking while stationary.
- Test tower tracking while driving.
- Only after those pass, test moving-shot lead behavior.
- Revisit ball-speed and time-of-flight assumptions if the turret consistently leads too much or too little.

## Good Turret Tuning Questions

When the turret is wrong, ask these questions in order:

- Is the physical turret pointed where the software thinks it is?
- Is the software commanding the angle we expected?
- Is the turret wrong for fixed presets, or only for tracking?
- Is the problem the turret, the drivetrain pose, or the vision pose?
- Is the error constant, directional, or motion-dependent?

These questions usually narrow the bug much faster than changing gains randomly.

## Helpful Dashboard Values For Turret And Tracking

Useful values already published by the robot include:

- `Robot X`
- `Robot Y`
- `Robot Angle`
- `Pigeon Angle`
- `Pose Distance`
- `estimated drive pose x`
- `estimated drive pose y`
- `estimated drive pose rotation`
- `Vision/Selected/Available`
- `Vision/Selected/Camera`
- `Vision/Right/Status`
- `Vision/Left/Status`
- `Vision/Right/TranslationErrorMeters`
- `Vision/Left/TranslationErrorMeters`

These are especially helpful when fixed turret commands work but field tracking does not.

## Drivetrain Commissioning Checklist

Use this after drivetrain repairs, gyro changes, or swerve tuning work.

1. Confirm all four modules report normally in Phoenix Tuner.
2. Confirm wheel azimuth directions look reasonable before driving.
3. Drive slowly and verify:
   - forward
   - strafe left/right
   - rotate
4. Confirm gyro heading agrees with the real robot heading.
5. Confirm the field pose moves smoothly in the expected direction.
6. Confirm alliance perspective behaves correctly after disable/enable.

If drivetrain field tracking is wrong, fix that before trying to tune turret tracking.

## Shooter Commissioning Checklist

Use this after shooter repairs, motor replacement, or table tuning.

1. Verify both shooter motors spin the intended direction.
2. Verify the follower motor mirrors the leader correctly.
3. Test fixed shooter presets before lerp-table tuning.
4. Test the lerp-table shot only after fixed-speed shots behave normally.
5. Tune one distance at a time and record the result.

## A Good "One Change At A Time" Rule

During tuning, change only one of these at once:

- sensor zero
- setpoint convention
- motor inversion
- PID gain
- feedforward gain
- interpolation table entry
- vision trust threshold

If several things are changed together, it becomes much harder to learn what
actually fixed the problem.

## Recommended Pit Workflow For A Turret Problem

1. Put the robot in a safe test condition.
2. Reboot with the turret physically staged correctly.
3. Test fixed presets.
4. Test rear-seam behavior.
5. Test tower tracking while stationary.
6. Test tower tracking while driving.
7. Test moving-shot lead last.

If the turret is still wrong after step 3, focus on:

- startup zero
- angle convention
- direction sign
- mechanical slip
- PID

If the turret is only wrong after step 5, focus on:

- drivetrain pose
- gyro heading
- alliance target
- vision reliability

## Final Reminder

A bad-looking turret shot does not automatically mean the turret PID is bad.

In this robot, a missed tracked shot can come from:

- wrong turret zero
- wrong drivetrain pose
- wrong gyro heading
- bad alliance selection
- poor vision data
- wrong shooter speed
- wrong lead assumptions

Start simple, isolate the subsystem, and only tune the next layer after the one
below it is behaving correctly.
