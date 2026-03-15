/*
 * Copyright (C) 2026 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import static frc.team3602.robot.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import frc.team3602.robot.Vision;
import frc.team3602.robot.Constants.ShooterConstants;
import frc.team3602.robot.subsystems.CommandSwerveDrivetrain;

public class ShooterSubsystem extends SubsystemBase {

    public CommandSwerveDrivetrain drivetrain;

    // Shooter Motors
    private final TalonFX shootermotor1;
    private final TalonFX shootermotor2;

   

    // Instantiating Classes
    public Vision vision;

    // Interpolation Table Instantiation
    public double shootLerpSpeed = 0.0;
    double shootShuffleSpeed = 0.0;
    public final InterpolatingDoubleTreeMap shootLerp = new InterpolatingDoubleTreeMap();
    // Feeding Motor
    // private static TalonFX feedermoter;

    // Constructor
    public ShooterSubsystem(Vision vision, CommandSwerveDrivetrain drivetrain) {
        shootermotor1 = new TalonFX(ShooterConstants.kShooterMotor1ID, "rio");
        shootermotor2 = new TalonFX(ShooterConstants.kShooterMotor2ID, "rio");
        this.vision = vision;
        this.drivetrain = drivetrain;
        // feedermoter = new TalonFX(ShooterConstants.kFeederMotorID);
        configShooterSubsys();
        SmartDashboard.putNumber("ShootSpeedInput", shootShuffleSpeed);
    }

    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);


    public double getVelocity() {
        return shootermotor1.getVelocity().getValueAsDouble();
    }

    // Go
    // public Command setShootSpeed() {
    //     return run(() -> {
    //         shootermotor1.set(-shootLerpSpeed);
    //         // shootermotor2.set(shootLerpSpeed); //shootLerpSpeed
    //     });
    // }`

    /**
     * Sends one shooter velocity target to the motor controllers.
     *
     * Phoenix keeps holding the last Motion Magic velocity target after we send it,
     * so this should be a one-shot command instead of a never-ending command. That
     * makes it safe to use inside command sequences for autonomous and timed shots.
     */
    public Command setShootVelocity(double rotationsPerSecond) {
        return runOnce(() -> {
            restoreFollowerControl();
            shootermotor1.setControl(m_request.withVelocity(rotationsPerSecond));
        });
    }

    /**
     * Sends the interpolated shooter velocity target to the shooter.
     *
     * We command only the leader motor here because motor 2 is configured to follow
     * it. Keeping one control path avoids the two motors fighting each other.
     */
    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
    public Command setShootVLerp() {
        return runOnce(() -> {
            restoreFollowerControl();
            shootermotor1.setControl(m_request.withVelocity(shootLerpSpeed));
        });
    }

    // public Command setShootVoltage(double shootVoltz) {
    //     return runOnce(() -> {
    //         shootermotor1.setVoltage(shootVoltz);
    //         // shootermotor2.setVoltage(-shootVoltz);
    //     });
    // }

    // public Command setFeederSpeed(double speed) {
    // return runOnce(() -> {
    // feedermoter.set(ShooterConstants.kFeederMotorSpeed);
    // });
    // }

    // Stop
    public Command stopShooter() {
        return runOnce(() -> {
            // Stop the leader using the same closed-loop path we use for normal
            // shooting, then immediately re-apply the follower request.
            //
            // This avoids sending a direct percent-output command to motor 2, which
            // could leave it no longer following the leader on the next shot.
            shootermotor1.setControl(m_request.withVelocity(0));
            restoreFollowerControl();
        });
    }

    // public Command stopFeeder(double feedStop) {
    // return runOnce(() -> {
    // feedermoter.set(0);
    // });
    // }
    // Periodic //six seven
    double angle;
    double distance;

     //If at speed
    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
    public boolean atSpeed() {
        return shootermotor1.getMotionMagicAtTarget().getValue();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Shooter1 Speed", ShooterConstants.kShooterSpeed);
        // SmartDashboard.putNumber("Shooter2 Speed", ShooterConstants.kShooterSpeed);
        // SmartDashboard.putNumber("Feeder Speed", ShooterConstants.kFeederMotorSpeed);
        distance = drivetrain.getDistanceToTarget();
        shootLerpSpeed = shootLerp.get(distance);
        // SmartDashboard.putNumber("Lerp Shoot Speed", shootLerpSpeed);
        // SmartDashboard.putNumber("Dist in side of shootSubsys", distance / 12);
        // SmartDashboard.getNumber("ShootSpeedInput", shootShuffleSpeed);
        SmartDashboard.putNumber("Shooter Lerp Speed", shootLerpSpeed);

    }

    private void configShooterSubsys() {       
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = .19; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        // set Motion Magic Velocity settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 600; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 6000; // Targ  et jerk of 4000 rps/s/s (0.1 seconds)

        // Set motor current limits
        var currentLimitConfigs = talonFXConfigs.CurrentLimits;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        currentLimitConfigs.StatorCurrentLimit = 60;
        currentLimitConfigs.SupplyCurrentLimit = 100;

        shootermotor1.getConfigurator().apply(talonFXConfigs);
        shootermotor2.getConfigurator().apply(talonFXConfigs);

        restoreFollowerControl();

        // Interpolation table config
        shootLerp.put(2.0, 31.0);
        shootLerp.put(3.0, 33.0);
        shootLerp.put(4.0, 35.0);
        shootLerp.put(5.0, 37.0);
        shootLerp.put(6.0, 39.0);
        shootLerp.put(7.0, 41.0);
        shootLerp.put(8.0, 43.0);
        shootLerp.put(9.0, 45.0);
        shootLerp.put(10.0, 47.0);
        shootLerp.put(11.0, 49.0);
        shootLerp.put(12.0, 51.0);
        shootLerp.put(13.0, 54.0);
        shootLerp.put(14.0, 56.0);
        shootLerp.put(15.0, 58.0);
        shootLerp.put(16.0, 60.0);
        shootLerp.put(17.0, 62.5);
        // Keep the final point monotonic until the team measures a better long-shot
        // speed on the practice field. The old 0.9 value would nearly stop the
        // shooter at the far end of the table.
        shootLerp.put(18.0, 64.5);


    }

    /**
     * Re-applies the follower request for shooter motor 2.
     *
     * The robot only intends to command the leader motor directly. Calling this
     * helper any time we change shooter state keeps motor 2 synchronized even after
     * stop commands or future tuning changes.
     */
    private void restoreFollowerControl() {
        // Motor 2 should always mirror motor 1.
        // We keep the follower setup in one helper so all shooter commands use the
        // same synchronization behavior.
        shootermotor2.setControl(new Follower(shootermotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

}
