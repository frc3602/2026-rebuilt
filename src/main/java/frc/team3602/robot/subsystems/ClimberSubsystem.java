package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.climberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private static TalonFX climbMotor;

    public ClimberSubsystem() {
        climbMotor = new TalonFX(climberConstants.kClimberMotorID);
    }

    public Command setClimbSpeed() {
        return runOnce(()-> 
        climbMotor.set(climberConstants.kClimberSpeed));
    }

    public Command stopClimb() {
        return runOnce(()->
        climbMotor.set(0));

    }
}
