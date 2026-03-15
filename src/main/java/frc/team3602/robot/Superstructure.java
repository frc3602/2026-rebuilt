/*
 * Copyright (C) 2026 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

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
    // These constants define the team's current simple autonomous shot.
    // Keeping them here makes the named PathPlanner commands easier to read and
    // easier for students to tune without digging through each command body.
    private static final double AUTON_BETA_SHOOTER_SPEED_RPS = -41.5;
    private static final double AUTON_BETA_READY_THRESHOLD_RPS = -41.25;
    private static final double AUTON_BETA_SPINDEXER_FEED_RPS = -30.0;
    private static final double AUTON_BETA_READY_TIMEOUT_SECONDS = 2.0;
    private static final double AUTON_BETA_FEED_TIME_SECONDS = 2.0;
    private static final double AUTON_TURRET_TRACK_SETTLE_SECONDS = 2.0;


    public IntakeSubsystem intakeSubsys;
    public ShooterSubsystem shooterSubsys;
    public SpindexerSubsystem spindexerSubsys;
    public TurretSubsystem turretSubsys;
    public CommandSwerveDrivetrain commandSwerveDrivetrainsubsys;
    public Vision vision;
    public PivotSubsystem pivotSubsys;
    public ClimberSubsystem climberSubsys;

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
                        spindexerSubsys.setFeedVelocity(-62.5).withTimeout(1.0)),
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
                turretSubsys.setAngle(0),
                // Set the shooter target once, then wait for the flywheel spin-up
                // time before feeding. This keeps the command sequence moving.
                shooterSubsys.setShootVelocity(ShooterConstants.kShooterFailsafeSpeed),
                Commands.waitSeconds(4),
                spindexerSubsys.setFeedVelocity(-62.5));
    }

    public Command stopShoot() {
        return Commands.sequence(shooterSubsys.stopShooter(),
                spindexerSubsys.stopSpindexer());

    }

    // Intake
    public Command dropPivot() {
        return pivotSubsys.smartDropPivot();
    }

    public Command stopIntake() {
        return pivotSubsys.dumbRaiseIntake().alongWith(intakeSubsys.stopIntake());
    }

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
     * Starts the beta autonomous shooter speed without feeding.
     *
     * This is useful when a PathPlanner auto wants to begin spinning the flywheel
     * while the robot is still finishing a short drive or a turret-alignment step.
     */
    public Command autonSpinUpBetaShot() {
        return shooterSubsys.setShootVelocity(AUTON_BETA_SHOOTER_SPEED_RPS);
    }

    /**
     * Waits until the shooter is close enough to the beta autonomous target speed.
     *
     * The timeout prevents the auto from getting stuck forever if the flywheel
     * never reaches speed because of battery sag, tuning, or a mechanical issue.
     */
    public Command autonWaitForBetaShotReady() {
        return Commands.waitUntil(() -> shooterSubsys.getVelocity() <= AUTON_BETA_READY_THRESHOLD_RPS)
                .withTimeout(AUTON_BETA_READY_TIMEOUT_SECONDS);
    }

    /**
     * Feeds one autonomous shot using the spindexer, then stops the feed motors.
     *
     * We keep feeding separate from spin-up so PathPlanner can decide exactly when
     * to release the fuel after the robot has finished moving and aiming.
     */
    public Command autonFeedBetaShot() {
        return Commands.sequence(
                spindexerSubsys.setFeedVelocity(AUTON_BETA_SPINDEXER_FEED_RPS)
                        .withTimeout(AUTON_BETA_FEED_TIME_SECONDS),
                spindexerSubsys.stopSpindexer());
    }

    /**
     * Stops the autonomous shooting hardware after a shot is complete.
     *
     * This gives PathPlanner a simple cleanup step so later auto actions do not
     * accidentally keep the shooter or spindexer running.
     */
    public Command autonStopShooting() {
        return Commands.sequence(
                spindexerSubsys.stopSpindexer(),
                shooterSubsys.stopShooter());
    }

    /**
     * Prepares the simple beta autonomous shot.
     *
     * This command starts the shooter, keeps the turret tracking the alliance
     * tower, and ends once the flywheel is ready or the timeout expires. It is a
     * good "next step after driving" block for simple autonomous routines.
     */
    public Command autonPrepareBetaShot() {
        return Commands.sequence(
                autonSpinUpBetaShot(),
                Commands.deadline(
                        autonWaitForBetaShotReady(),
                        turretSubsys.trackAllianceTower()));
    }

    /**
     * Runs the full beta autonomous shot as one reusable macro.
     *
     * This keeps the original one-command auto behavior available even after we
     * split the sequence into smaller named pieces for PathPlanner.
     */
    public Command autonRunBetaShot() {
        return Commands.sequence(
                autonPrepareBetaShot(),
                autonFeedBetaShot(),
                autonStopShooting());
    }

    /**
     * Continuously tracks the current alliance tower with the turret.
     *
     * This command is intended for autonomous use while the drivetrain is doing
     * something else, such as following a path. Because the command keeps updating
     * the turret every loop, it does not end by itself and should be used in
     * parallel, as a PathPlanner event command, or with a timeout.
     */
    public Command autonTrackTower() {
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
    public Command autonTrackTowerShort() {
        return autonTrackTower().withTimeout(AUTON_TURRET_TRACK_SETTLE_SECONDS);
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
