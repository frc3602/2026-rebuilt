package frc.team3602.robot;

/**
 * Provides a small wrapper around the remaining non-turret Limelight reads.
 *
 * The robot's main AprilTag pose pipeline now lives in {@code Limelight_Pose},
 * so this class is only responsible for simple direct reads from the primary
 * camera that other subsystems still use.
 */
public class Vision {

    /**
     * Returns the primary Limelight horizontal error in degrees.
     *
     * Positive and negative values tell the caller which side of the camera the
     * target appears on.
     */
    public double getTX() {
        return LimelightHelpers.getTX("limelight-primary");
    }

    /**
     * Returns whether the primary Limelight currently sees a valid target.
     *
     * This is still used by code paths that want a quick "target visible" check
     * without requesting a full pose estimate.
     */
    public boolean getHasTarget() {
        return LimelightHelpers.getTV("limelight-primary");
    }
}
