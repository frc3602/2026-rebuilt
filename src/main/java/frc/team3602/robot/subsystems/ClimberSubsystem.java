package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climberMotor = new TalonFX(/*ClimberConstants.kClimberMotorID*/20);

    private double pivotSetPoint = 0.0;
    public double pidEffort;
    public double ffEffort;
    public double totalEffort;

    public ClimberSubsystem() {
        var ClimbMotorConfig = new MotorOutputConfigs();
        var ClimbLimitConfig = new CurrentLimitsConfigs();
    }

    public double climberPosition() {
       return (climberMotor.getRotorPosition().getValueAsDouble() * (Math.PI * 2.15) /  20.25);
    }

    private final PIDController climbController = new PIDController(43.19, 0, 0.19);
    private final ArmFeedforward climbFeedforward = new ArmFeedforward(0, 0.69, 15.79, 0.07);

    public Command setClimbHeight(double setpoint) {
        return runOnce(() ->  pivotSetPoint = setpoint);
    }

   //Climber
    public Command raiseClimber(){
        return this.setClimbHeight(20);
    }

    public Command lowerClimber(){
        return this.setClimbHeight(5);
    }

    public Command setPosition() {
        return run(() -> {
            var pidEffort = climbController.calculate(climberPosition(), pivotSetPoint);
            this.pidEffort = pidEffort;

            var ffEffort = climbFeedforward.calculate(0, 0);
            this.ffEffort = ffEffort;

            var totalEffort = pidEffort + ffEffort;
            this.totalEffort = totalEffort;

            climberMotor.setVoltage(totalEffort);
        });
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("PID Effort", ffEffort);
        // SmartDashboard.putNumber("Feedforward Effort", ffEffort);
        // SmartDashboard.putNumber("Total Effort", totalEffort);

        // SmartDashboard.putNumber("Climber Setpoint", pivotSetPoint);
        // SmartDashboard.putNumber("Climber Measurement", climberPosition());
    }
}