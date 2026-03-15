package frc.team3602.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.spindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {
    // The spindexer wheel is about 4 inches in diameter while the transfer/X44
    // wheel is about 1 inch in diameter. To keep their surface speeds matched,
    // the smaller transfer wheel needs to spin about 4x faster than the spindexer.
    private static final double TRANSFER_TO_SPINDEXER_DIAMETER_RATIO = 0.25;

    /* Motors */

    private final TalonFX spindexerMotor;
    private final TalonFX receiveMotor;

    /* Constructor */

    /**
     * Creates both feed motors and applies the shared closed-loop config.
     *
     * The spindexer and receive motor always work as a pair, so we configure them
     * together in one place.
     */
    public SpindexerSubsystem() {

        spindexerMotor = new TalonFX(spindexerConstants.kSpindexerMotorID, "rio");
        receiveMotor = new TalonFX(spindexerConstants.kReceiveMotorID, "rio");
        configSpindexerSubsys();
    }

    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

    /* Commands */

    /**
     * Applies one closed-loop velocity request to both feed motors.
     *
     * Keeping this in one helper makes sure every scoring path uses the same ratio
     * between the main spindexer wheel and the smaller transfer/X44 wheel.
     */
    private void applyFeedVelocityRequest(double rotationsPerSecond) {
        receiveMotor.setControl(
                m_request.withVelocity(rotationsPerSecond / TRANSFER_TO_SPINDEXER_DIAMETER_RATIO));
        spindexerMotor.setControl(m_request.withVelocity(rotationsPerSecond));
    }

    /**
     * Stops both feed motors immediately.
     */
    private void stopFeedMotors() {
        spindexerMotor.set(0);
        receiveMotor.set(0);
    }

    /**
     * Stops the spindexer path with a one-shot command.
     *
     * This is used as cleanup after timed feeds and on button release in teleop.
     */
    public Command stopSpindexer() {
        return runOnce(() -> {
            stopFeedMotors();
        });
    }

    /**
     * Runs the spindexer and transfer motor together in closed-loop velocity mode.
     *
     * This is the preferred way to move fuel through the spindexer system because
     * both motors hold a requested velocity instead of relying on battery-dependent
     * percent output. The receive motor is automatically scaled so the small X44
     * transfer wheel stays close to the same surface speed as the larger spindexer
     * wheel.
     */
    public Command setFeedVelocity(double rotationsPerSecond) {
        return runEnd(
                () -> applyFeedVelocityRequest(rotationsPerSecond),
                this::stopFeedMotors);
    }

    /**
     * Runs the feed path only while a separate ready condition is true.
     *
     * This is useful for a single-button shot command. The robot can keep aiming
     * and spinning up first, then begin feeding automatically once the shooter is
     * ready.
     */
    public Command setFeedVelocityWhen(BooleanSupplier shouldFeed, double rotationsPerSecond) {
        return runEnd(() -> {
            if (shouldFeed.getAsBoolean()) {
                applyFeedVelocityRequest(rotationsPerSecond);
            } else {
                stopFeedMotors();
            }
        }, this::stopFeedMotors);
    }

    /**
     * Reverses the spindexer path to help clear jams.
     *
     * We keep this as a simple percent-output command because reverse is used as a
     * short manual recovery action rather than a carefully metered scoring feed.
     */
    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
    public Command setReverseSpindexerReceive() {
        return runOnce(() -> {
            receiveMotor.set(spindexerConstants.kRecieveFuelSpeed);
            spindexerMotor.set(-spindexerConstants.kSpindexerMotorSpeed);
        });
    }

    /* Periodic */

    @Override
    public void periodic() {
        // SmartDashboard.putData("spindexer sped", (Sendable) spindexerMotor.getVelocity());
        // SmartDashboard.putData("spindexer speed", (Sendable) spindexerMotor.getMotorVoltage());
    }

    /**
     * Applies the current closed-loop config shared by the spindexer and receive
     * motors.
     *
     * Both motors use Motion Magic velocity control so feed speed stays more
     * consistent as battery voltage changes during a match.
     */
    private void configSpindexerSubsys() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        
        // Slot0 holds the basic feedforward and PID gains for velocity control.
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

        // Protect both motors with the same current limits so the feed path behaves
        // predictably and is easier to tune.
        var currentLimitConfigs = talonFXConfigs.CurrentLimits;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        currentLimitConfigs.StatorCurrentLimit = 40;
        currentLimitConfigs.SupplyCurrentLimit = 60;

        // Apply the same velocity-control setup to both motors. Their requested
        // speeds differ later in applyFeedVelocityRequest(), but the control style is
        // shared.
        spindexerMotor.getConfigurator().apply(talonFXConfigs);
        receiveMotor.getConfigurator().apply(talonFXConfigs);
    }

}
