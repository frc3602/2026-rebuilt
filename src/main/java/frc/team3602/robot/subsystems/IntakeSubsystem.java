package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.IntakeConstants;
import frc.team3602.robot.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase{
    //variables
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorID, "rio");
    
    //constructor
    public IntakeSubsystem(){
        configIntakeSubsys();
    }

    private final PIDController pivotPID = new PIDController(0.15, 0.0,0.0);
    private final PIDController pivotFollowerPID = new PIDController(0.15,0.0, 0.0);

    //EAT
    public Command setIntakeSpeed() {
        return runOnce(() -> 
            intakeMotor.set(IntakeConstants.kIntakeMotorSpeed));
    
    }

    //STOP
    public Command stopIntake() {
        return runOnce(() ->
        intakeMotor.set(0));
    }

    //VOMIT
    public Command reverseIntake() {
        return runOnce(() ->
        intakeMotor.set(-IntakeConstants.kIntakeMotorSpeed));
    }

    @Override
    public void periodic() {

    }

    private void configIntakeSubsys() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

                 // Set motor current limits
        var currentLimitConfigs = talonFXConfigs.CurrentLimits;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        // currentLimitConfigs.StatorCurrentLimit = 60;
        // currentLimitConfigs.SupplyCurrentLimit = 60;

        intakeMotor.getConfigurator().apply(talonFXConfigs);

    }
}

