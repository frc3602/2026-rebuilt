/*
 * Copyright (C) 2026 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import static frc.team3602.robot.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants;
import frc.team3602.robot.Constants.ShooterConstants;
import frc.team3602.robot.Vision;


public class ShooterSubsystem extends SubsystemBase {

    double voltage;
    public final Vision vision = new Vision();

    // Shooter Motors
    private static TalonFX shootermotor1;
    private static TalonFX shootermotor2;
    private static TalonFX hoodmotor;
    // Feeding Motor
    private static TalonFX feedermoter;

    //PID
    private PIDController hoodController = new PIDController(0, 0, 0);

    // Constructor
    public ShooterSubsystem() {
        shootermotor1 = new TalonFX(ShooterConstants.kShooterMotor1ID);
        shootermotor2 = new TalonFX(ShooterConstants.kShooterMotor2ID);
        feedermoter = new TalonFX(ShooterConstants.kFeederMotorID);
        hoodmotor = new TalonFX(ShooterConstants.kHoodMotorID);

    }

    // Go
    public Command setShootSpeed(double shootSpeed) {
        return runOnce(() -> {
            shootermotor1.set(ShooterConstants.kShooterSpeed);
            shootermotor2.set(ShooterConstants.kShooterSpeed); 
        });
    }

    public Command setFeederSpeed(double speed) {
        return runOnce(() -> {
            feedermoter.set(ShooterConstants.kFeederMotorSpeed);
        });
    }

    // Stop
    public Command stopShooter(double shootStop) {
        return runOnce(() -> {
            shootermotor1.set(0);
        });
    }

    public Command stopFeeder(double feedStop) {
        return runOnce(() -> {
            feedermoter.set(0);
        });
    }

    public Command hoodConstraints() {
        return run(() -> {

            if (vision.getTurretHasTarget()) {
            
            }
            if (voltage > ShooterConstants.hoodMaxVolt){
                voltage = ShooterConstants.hoodMaxVolt;
            }
            else if (voltage < -ShooterConstants.hoodMaxVolt){
                voltage = -ShooterConstants.hoodMaxVolt;
            }
            hoodmotor.setVoltage(voltage);
        }

        );

    }

    // Periodic
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter1 Speed", ShooterConstants.kShooterSpeed);
        SmartDashboard.putNumber("Shooter2 Speed", ShooterConstants.kShooterSpeed);
        SmartDashboard.putNumber("Feeder Speed", ShooterConstants.kFeederMotorSpeed);
    }
}
