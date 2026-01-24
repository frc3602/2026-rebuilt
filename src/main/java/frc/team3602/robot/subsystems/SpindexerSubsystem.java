package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.spindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {

    //Added stuff for receiving motor incase we use two

    /* Motors */

    private static TalonFX spindexerMotor;
    // private static TalonFX receiveMotor;

    /* Constructor */

    public SpindexerSubsystem() {

        spindexerMotor = new TalonFX(spindexerConstants.kSpindexerMotorID);
        // receiveMotor = new TalonFX(spindexerConstants.kReceiveMotorID);
    }

    /* Commands */

    public Command transferFuel() {
        return runOnce(() -> {
            spindexerMotor.set(spindexerConstants.kSpindexerMotorSpeed);
        });
    }

    public Command stopSpindexer() {
        return runOnce(() -> {
            spindexerMotor.set(spindexerConstants.kStopSpindexerMotorSpeed);
        });
    }

    // public Command receiveFuel() {
    //     return runOnce(() -> {
    //         receiveMotor.set(spindexerConstants.kRecieveFuelSpeed);
    //     });
    // }

    /* Periodic */

    @Override
    public void periodic() {
        SmartDashboard.putData("spindexer speed", (Sendable) spindexerMotor.getVelocity());
        SmartDashboard.putData("spindexer speed", (Sendable) spindexerMotor.getMotorVoltage());
    }

}