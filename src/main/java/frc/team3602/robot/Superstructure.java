/*
 * Copyright (C) 2026 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3602.robot.Constants.ShooterConstants;
import frc.team3602.robot.Constants.spindexerConstants;
import frc.team3602.robot.subsystems.ClimberSubsystem;
import frc.team3602.robot.subsystems.CommandSwerveDrivetrain;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;
import frc.team3602.robot.subsystems.SpindexerSubsystem;
import frc.team3602.robot.subsystems.TurretSubsystem;

public class Superstructure {

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
        this.pivotSubsys = pivotsubsys;
    }

    /* Score Commands */
    public Command shootBall1() {
        return Commands.parallel(
                turretSubsys.aimCommand(),
                Commands.sequence(

                        shooterSubsys.setShootVelocity(-62.5).withTimeout(1.7),
                        spindexerSubsys.setFeedVelocity(-62.5)));
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
    public Command autonIntake() {
        return Commands.sequence(
            pivotSubsys.dumbDropIntake()
        );
    }

    // public Command autonShoot() {
    //     return Commands.sequence(
    //             shooterSubsys.setShootVelocity(ShooterConstants.kShooterFailsafeSpeed),
    //             spindexerSubsys.setFeedVelocity(-30));
    // }

    public Command autonShootBeta() {
        return Commands.parallel(
                shooterSubsys.setShootVelocity(-41.25).andThen(Commands.waitUntil(() -> shooterSubsys.getVelocity() >= -41.25)).withTimeout(5).andThen(shooterSubsys.stopShooter()),  
                spindexerSubsys.setFeedVelocity(-30.0)).withTimeout(5).andThen(spindexerSubsys.setFeedVelocity(0));               
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
