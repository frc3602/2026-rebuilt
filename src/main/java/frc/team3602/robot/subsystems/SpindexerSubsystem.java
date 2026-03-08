package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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
    private static TalonFX receiveMotor;

    /* Constructor */

    public SpindexerSubsystem() {

        spindexerMotor = new TalonFX(spindexerConstants.kSpindexerMotorID, "rio");
        receiveMotor = new TalonFX(spindexerConstants.kReceiveMotorID, "rio");
        configSpindexerSubsys();
    }

    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

    /* Commands */


    public Command stopSpindexer() {
        return runOnce(() -> {
            spindexerMotor.set(0);
            receiveMotor.set(0);
        });
    }

    public Command setSpindexerReceive() {
        return runOnce(() -> {
            receiveMotor.set(-spindexerConstants.kRecieveFuelSpeed);
            spindexerMotor.set(spindexerConstants.kSpindexerMotorSpeed);
        });
    }

    public Command setFeedVelocity(double rotationsPerSecond) {
        return run(() -> {
            receiveMotor.setControl(m_request.withVelocity(-rotationsPerSecond));
            spindexerMotor.setControl(m_request.withVelocity(rotationsPerSecond));
        });

    }

    public Command setFasterSpindexerReceive() {
        return runOnce(() -> {
            receiveMotor.set(-spindexerConstants.kRecieveFuelSpeed);
            spindexerMotor.set(spindexerConstants.kSpindexerMotorSpeed);
        });
    }

    /* Periodic */

    @Override
    public void periodic() {
        // SmartDashboard.putData("spindexer sped", (Sendable) spindexerMotor.getVelocity());
        // SmartDashboard.putData("spindexer speed", (Sendable) spindexerMotor.getMotorVoltage());
    }

    private void configSpindexerSubsys() {
                TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.1; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        // set Motion Magic Velocity settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 6000; // Targ  et jerk of 4000 rps/s/s (0.1 seconds)

                // Set motor current limits
        var currentLimitConfigs = talonFXConfigs.CurrentLimits;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        currentLimitConfigs.StatorCurrentLimit = 40;
        currentLimitConfigs.SupplyCurrentLimit = 60;

        spindexerMotor.getConfigurator().apply(talonFXConfigs);
        receiveMotor.getConfigurator().apply(talonFXConfigs);
    }

}