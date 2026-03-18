/*
 * Copyright (C) 2026 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3602.robot.Constants.ShooterConstants;
import frc.team3602.robot.subsystems.ClimberSubsystem;
import frc.team3602.robot.subsystems.CommandSwerveDrivetrain;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;
import frc.team3602.robot.subsystems.SpindexerSubsystem;
import frc.team3602.robot.subsystems.TurretSubsystem;

public class Superstructure {
    // These constants define the team's current simple autonomous tower shot.
    // Keeping them here makes the named PathPlanner commands easier to read and
    // easier for students to tune without digging through each command body.
    private static final double AUTON_TOWER_SHOT_SHOOTER_SPEED_RPS = -41.5;
    private static final double AUTON_TOWER_SHOT_READY_THRESHOLD_RPS = -41.25;
    private static final double AUTON_TOWER_SHOT_READY_TIMEOUT_SECONDS = 2.0;
    private static final double AUTON_TOWER_SHOT_FEED_TIME_SECONDS = 2.0;
    private static final double AUTON_TURRET_TRACK_SETTLE_SECONDS = 2.0;
    // The fallback shot still points to the rear of the robot, but turret command
    // APIs now use WPILib's signed convention where 180 degrees means "backward."
    private static final double FAILSAFE_TURRET_ANGLE_DEGREES = 180.0;
    private static final double FAILSAFE_SHOOTER_SPINUP_SECONDS = 4.0;
    private static final double FAILSAFE_FEED_TIME_SECONDS = 1.0;
    private static final double TRACKED_LERP_READY_SHOOTER_TOLERANCE_RPS = 2.5;
    private static final double TRACKED_LERP_READY_TURRET_TOLERANCE_DEGREES = 2.5;
    private static final double TRACKED_LERP_READY_DEBOUNCE_SECONDS = 0.05;


    public IntakeSubsystem intakeSubsys;
    public ShooterSubsystem shooterSubsys;
    public SpindexerSubsystem spindexerSubsys;
    public TurretSubsystem turretSubsys;
    public CommandSwerveDrivetrain commandSwerveDrivetrainsubsys;
    public Vision vision;
    public PivotSubsystem pivotSubsys;
    public ClimberSubsystem climberSubsys;
    private final Debouncer trackedLerpReadyDebouncer = new Debouncer(TRACKED_LERP_READY_DEBOUNCE_SECONDS);
    private boolean trackedLerpShotReadyLatched = false;

    public Superstructure(IntakeSubsystem intakeSubsys, ShooterSubsystem shooterSubsys,
            SpindexerSubsystem spindexerSubsys,
            TurretSubsystem turretSubsys, CommandSwerveDrivetrain commandSwerveDrivetrainSubsys,
            PivotSubsystem pivotsubsys) {
        this.intakeSubsys = intakeSubsys;
        this.shooterSubsys = shooterSubsys;
        this.spindexerSubsys = spindexerSubsys;
        this.turretSubsys = turretSubsys;
        this.commandSwerveDrivetrainsubsys = commandSwerveDrivetrainSubsys;
        this.pivotSubsys = pivotsubsys;
    }

    /* Score Commands */
    public Command shootBall1() {
        return Commands.deadline(
                Commands.sequence(
                        // Set the shooter target once and give the flywheel time to
                        // spin up before feeding.
                        shooterSubsys.setShootVelocity(-62.5),
                        Commands.waitSeconds(1.7),
                        // Feed for a short fixed window so this command can finish on
                        // its own in teleop or autonomous.
                        spindexerSubsys.setFeedVelocityScaledFromShooter(shooterSubsys::getVelocity)
                            .withTimeout(1.0)),
                turretSubsys.aimCommand())
                .andThen(stopShoot());
    }

    // public Command shootBall2() {
    // return Commands.parallel(
    // turretSubsys.track(),
    // Commands.sequence(
    // shooterSubsys.setShootVelocity(-57.5).withTimeout(2).andThen(
    // spindexerSubsys.setFeedVelocity(-57.5))

    // ));
    // }

    public Command shootFailsafe() {
        return Commands.sequence(
                turretSubsys.setAngle(FAILSAFE_TURRET_ANGLE_DEGREES),
                // Set the shooter target once, then wait for the flywheel spin-up
                // time before feeding.
                //
                // We give the feed step its own timeout and cleanup so this routine
                // can safely be reused in teleop or autonomous without hanging
                // forever at the final command.
                shooterSubsys.setShootVelocity(ShooterConstants.kShooterFailsafeSpeed),
                Commands.waitSeconds(FAILSAFE_SHOOTER_SPINUP_SECONDS),
                spindexerSubsys.setFeedVelocityScaledFromShooter(shooterSubsys::getVelocity)
                    .withTimeout(FAILSAFE_FEED_TIME_SECONDS),
                stopShoot());
    }

    /**
     * Returns whether the robot is ready to release the tracked lerp shot.
     *
     * We wait for both the flywheel and the turret:
     * - shooter must be near the current lerp-table speed
     * - turret must be near its current tower-tracking setpoint
     *
     * This prevents the single-button shot from feeding immediately when the button
     * is first pressed.
     */
    private boolean isTrackedLerpShotRawReady() {
        boolean shooterReady = shooterSubsys.isNearLerpVelocity(TRACKED_LERP_READY_SHOOTER_TOLERANCE_RPS);
        double turretAimErrorDegrees = Math.abs(turretSubsys.getTurretAimErrorDegrees());
        boolean turretReady = turretAimErrorDegrees <= TRACKED_LERP_READY_TURRET_TOLERANCE_DEGREES;

        // Publish the two readiness checks separately so the team can see whether
        // the B-button shot is waiting on flywheel speed or turret angle.
        SmartDashboard.putBoolean("TrackedShot/ShooterReady", shooterReady);
        SmartDashboard.putBoolean("TrackedShot/TurretReady", turretReady);
        SmartDashboard.putNumber("TrackedShot/TurretAimErrorDegrees", turretAimErrorDegrees);

        return shooterReady && turretReady;
    }

    /**
     * Resets the latched readiness state used by the combined tracked shot.
     *
     * We call this when the command starts and when it ends so each new shot has to
     * earn readiness again instead of inheriting a stale state from the previous
     * button press.
     */
    private void resetTrackedLerpShotState() {
        trackedLerpShotReadyLatched = false;
        trackedLerpReadyDebouncer.calculate(false);
        SmartDashboard.putBoolean("TrackedShot/ReadyLatched", false);
    }

    /**
     * Updates the tracked-shot readiness latch.
     *
     * The debouncer filters out tiny readiness spikes, and the latch keeps the feed
     * path running once the robot has committed to the shot. This avoids the feed
     * and driver rumble chattering on and off if flywheel speed dips briefly when
     * fuel first enters the shooter.
     */
    private boolean updateTrackedLerpShotReadyState() {
        if (!trackedLerpShotReadyLatched && trackedLerpReadyDebouncer.calculate(isTrackedLerpShotRawReady())) {
            trackedLerpShotReadyLatched = true;
        }

        SmartDashboard.putBoolean("TrackedShot/ReadyLatched", trackedLerpShotReadyLatched);
        return trackedLerpShotReadyLatched;
    }

    /**
     * Returns the public latched readiness state for the tracked shot.
     *
     * Driver rumble and any future dashboard indicator should follow the same
     * latched state that actually controls feeding.
     */
    public boolean isTrackedLerpShotReady() {
        return trackedLerpShotReadyLatched;
    }

    /**
     * Runs a single-button tracked tower shot using the shooter lerp table.
     *
     * While the command is held:
     * - the turret tracks the current alliance tower
     * - the shooter continuously follows the current lerp-table target
     * - the feed path stays off until the shot is ready
     * - once ready, the spindexer and transfer run until the button is released
     */
    public Command shootTrackedLerpShot() {
        return Commands.parallel(
                turretSubsys.trackAllianceTower(),
                shooterSubsys.holdShootVLerp(),
            spindexerSubsys.setFeedVelocityScaledFromShooterWhen(
                this::updateTrackedLerpShotReadyState,
                shooterSubsys::getVelocity))
                .beforeStarting(this::resetTrackedLerpShotState)
                .finallyDo(interrupted -> resetTrackedLerpShotState());
    }

    public Command stopShoot() {
        return Commands.sequence(shooterSubsys.stopShooter(),
                spindexerSubsys.stopSpindexer());

    }

    /**
     * Continuously tracks the current alliance tower with the turret.
     *
     * Keeping this small wrapper in the superstructure makes operator bindings and
     * autonomous code read the same way even though the turret subsystem owns the
     * actual aiming math.
     */
    public Command trackAllianceTower() {
        return turretSubsys.trackAllianceTower();
    }

    // Intake
    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
    public Command dropPivot() {
        return pivotSubsys.smartDropPivot();
    }

    public Command stopIntake() {
        return pivotSubsys.dumbRaiseIntake().alongWith(intakeSubsys.stopIntake());
    }

    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
    public Command outakeBall() {
        return intakeSubsys.reverseIntake().withTimeout(.2);
    }

    // Auton
    public Command autonLowerIntake() {
        return Commands.sequence(
            pivotSubsys.dumbDropIntake()
        );
    }

    /**
     * Puts the intake system into its normal autonomous collection state.
     *
     * This is a convenience macro for PathPlanner when an auto almost always wants
     * the same two actions together:
     * - send the pivot toward the lowered intake preset
     * - start the intake roller
     *
     * The command ends immediately after those requests are sent. The pivot keeps
     * moving because its default hold command continues driving toward the lowered
     * setpoint, and the intake roller keeps running until a later stop command.
     */
    public Command autonRunIntake() {
        return Commands.sequence(
                autonLowerIntake(),
                autonStartIntake());
    }

    /**
     * Raises or stows the intake path for autonomous.
     *
     * We keep this separate from autonStopIntake() because some autos may want to
     * stop the roller first, finish driving, and only then lift the mechanism for a
     * shot or a protected return path.
     */
    public Command autonRaiseIntake() {
        return pivotSubsys.dumbRaiseIntake();
    }

    /**
     * Starts only the intake roller for autonomous.
     *
     * This is intentionally separate from pivot motion so PathPlanner can decide
     * whether lowering the intake, starting the roller, and driving should happen
     * together or in separate steps.
     */
    public Command autonStartIntake() {
        return intakeSubsys.setIntakeSpeed();
    }

    /**
     * Stops only the intake roller for autonomous.
     *
     * We keep this separate from any pivot-stow behavior so autos can stop the
     * roller without unexpectedly moving the mechanism.
     */
    public Command autonStopIntake() {
        return intakeSubsys.stopIntake();
    }

    // public Command autonShoot() {
    //     return Commands.sequence(
    //             shooterSubsys.setShootVelocity(ShooterConstants.kShooterFailsafeSpeed),
    //             spindexerSubsys.setFeedVelocity(-30));
    // }

    /**
     * Starts the autonomous tower-shot shooter speed without feeding.
     *
     * This is useful when a PathPlanner auto wants to begin spinning the flywheel
     * while the robot is still finishing a short drive or a turret-alignment step.
     */
    public Command autonStartShooter() {
        return shooterSubsys.setShootVelocity(AUTON_TOWER_SHOT_SHOOTER_SPEED_RPS);
    }

    /**
     * Waits until the shooter is close enough to the autonomous tower-shot speed.
     *
     * The timeout prevents the auto from getting stuck forever if the flywheel
     * never reaches speed because of battery sag, tuning, or a mechanical issue.
     */
    public Command autonWaitForShooterReady() {
        return Commands.waitUntil(() -> shooterSubsys.getVelocity() <= AUTON_TOWER_SHOT_READY_THRESHOLD_RPS)
                .withTimeout(AUTON_TOWER_SHOT_READY_TIMEOUT_SECONDS);
    }

    /**
     * Waits until the autonomous tower shot is ready to release.
     *
     * We require both:
     * - shooter at the expected autonomous speed
     * - turret settled at its currently requested angle
     *
     * Keeping this as one helper prevents PathPlanner sequences from advancing to
     * feed before turret aim has actually finished settling.
     */
    public Command autonWaitForTowerShotReady() {
        return Commands.waitUntil(() -> {
            boolean shooterReady = shooterSubsys.getVelocity() <= AUTON_TOWER_SHOT_READY_THRESHOLD_RPS;
            boolean turretReady = turretSubsys.isAtRequestedAngle();
            return shooterReady && turretReady;
        }).withTimeout(AUTON_TOWER_SHOT_READY_TIMEOUT_SECONDS);
    }

    /**
     * Feeds one autonomous shot using the spindexer, then stops the feed motors.
     *
     * We keep feeding separate from spin-up so PathPlanner can decide exactly when
     * to release the fuel after the robot has finished moving and aiming.
     */
    public Command autonFireShot() {
        return Commands.sequence(
            spindexerSubsys.setFeedVelocityScaledFromShooter(shooterSubsys::getVelocity)
                        .withTimeout(AUTON_TOWER_SHOT_FEED_TIME_SECONDS),
                spindexerSubsys.stopSpindexer());
    }

    /**
     * Stops the autonomous shooting hardware after a shot is complete.
     *
     * This gives PathPlanner a simple cleanup step so later auto actions do not
     * accidentally keep the shooter or spindexer running.
     */
    public Command autonStopShooter() {
        return Commands.sequence(
                spindexerSubsys.stopSpindexer(),
                shooterSubsys.stopShooter());
    }

    /**
     * Prepares the simple autonomous tower shot.
     *
     * This command starts the shooter, keeps the turret tracking the alliance
         * tower, and ends once both flywheel speed and turret aim are ready (or the
         * timeout expires). It is a good "next step after driving" block for simple
         * autonomous routines.
     */
    public Command autonPrepareTowerShot() {
        return Commands.sequence(
                autonStartShooter(),
                Commands.deadline(
                autonWaitForTowerShotReady(),
                        turretSubsys.trackAllianceTower()));
    }

    /**
     * Runs the full simple tower shot as one reusable macro.
     *
     * This keeps the original one-command auto behavior available even after we
     * split the sequence into smaller named pieces for PathPlanner.
     */
    public Command autonShootTower() {
        return Commands.sequence(
                autonPrepareTowerShot(),
                autonFireShot(),
                autonStopShooter());
    }

    /**
     * Continuously tracks the current alliance tower with the turret.
     *
     * This command is intended for autonomous use while the drivetrain is doing
     * something else, such as following a path. Because the command keeps updating
     * the turret every loop, it does not end by itself and should be used in
     * parallel, as a PathPlanner event command, or with a timeout.
     */
    public Command autonAimTower() {
        return turretSubsys.trackAllianceTower();
    }

    /**
     * Tracks the current alliance tower for a short fixed time.
     *
     * This version is easier to drop into a sequential autonomous routine because it
     * ends on its own instead of running forever. The timeout is intentionally short
     * so the turret gets time to aim without blocking the rest of the auto for too
     * long.
     */
    public Command autonAimTowerShort() {
        return autonAimTower().withTimeout(AUTON_TURRET_TRACK_SETTLE_SECONDS);
    }






    // Shooter
    // public Command ShootBall() {
    // return
    // shooterSubsys.setShootSpeed(ShooterConstants.kFeederMotorSpeed).withTimeout(1);
    // }
    // public Command FeedBall() {
    // return
    // shooterSubsys.setFeederSpeed(ShooterConstants.kFeederMotorSpeed).withTimeout(1);
    // }

    // //Turret

    // //Spindexer
    // public Command RunSpindexer() {
    // return
    // spindexerSubsys.setSpindexerSpeed(spindexerConstants.kSpindexerMotorSpeed).withTimeout(1);
    // }

    // public Command ReceiveBall() {
    // return
    // spindexerSubsys.setSpindexerSpeed(spindexerConstants.kRecieveFuelSpeed).withTimeout(1);
    // }
}
