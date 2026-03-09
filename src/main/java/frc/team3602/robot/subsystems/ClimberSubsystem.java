package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climberMotor = new TalonFX(ClimberConstants.kClimberMotorID);

    public ClimberSubsystem() {
        var ClimbMotorConfig = new MotorOutputConfigs();
        var ClimbLimitConfig = new CurrentLimitsConfigs();
    }

    public double climberPosition() {
       return (climberMotor.getRotorPosition().getValueAsDouble());
    }

    private final PIDController climbController = new PIDController(0, 0, 0);
    private final ArmFeedforward climbFeedforward = new ArmFeedforward(0, 0, 0, 0);

    private double climbTargetPosition = 5;

    // public Command setClimbHeight(double setpoint) {
    //     return runOnce(() -> climbTargetPosition = setpoint);
    // }
}
