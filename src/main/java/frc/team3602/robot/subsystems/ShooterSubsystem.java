/*
 * Copyright (C) 2026 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.team3602.robot.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import frc.team3602.robot.Vision;
import frc.team3602.robot.Constants.ShooterConstants;
import frc.team3602.robot.subsystems.CommandSwerveDrivetrain;

public class ShooterSubsystem extends SubsystemBase {

    public CommandSwerveDrivetrain drivetrain;

    // Shooter Motors
    private static TalonFX shootermotor1;
    private static TalonFX shootermotor2;

   

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
        shootermotor2.setControl(new Follower(shootermotor1.getDeviceID(), MotorAlignmentValue.Opposed));
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

    public Command setShootVelocity(double rotationsPerSecond) {
        return run(() -> {
            shootermotor1.setControl(m_request.withVelocity(rotationsPerSecond));
            shootermotor2.setControl(m_request.withVelocity(-rotationsPerSecond));
        });
    }

    public Command setShootVLerp() {
        return run(() -> {
            shootermotor1.setControl(m_request.withVelocity(shootLerpSpeed));
        }

        );

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
            shootermotor1.set(0);
            shootermotor2.set(0);
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
        SmartDashboard.putNumber("turretAngle", angle);

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
        shootLerp.put(18.0, 0.9);


    }

}
