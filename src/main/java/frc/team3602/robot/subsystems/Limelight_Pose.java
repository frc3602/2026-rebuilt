// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3602.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.LimelightHelpers;
import frc.team3602.robot.LimelightHelpers.PoseEstimate;
import frc.team3602.robot.LimelightHelpers.RawFiducial;

public class Limelight_Pose extends SubsystemBase {
  /**
   * This small helper object stores how one camera frame was evaluated.
   *
   * We keep the data together so students can see the full decision in one place:
   * what the camera saw, whether we accepted it, which pose mode we chose, and how
   * much we trust that pose update.
   */
  private static class CameraMeasurementDecision {
    public String cameraName = "Unknown";
    public boolean processedFreshFrame = false;
    public boolean acceptedMeasurement = false;
    public boolean usingMegaTag1 = false;
    public boolean usingMegaTag2 = false;
    public PoseEstimate selectedEstimate = null;
    public double xyStdDev = 0.0;
    public double thetaStdDev = LARGE_ROTATION_STD_DEV;
    public int tagCount = 0;
    public double avgTagArea = 0.0;
    public double avgTagDist = 0.0;
    public double tagSpan = 0.0;
    public double maxAmbiguity = 0.0;
    public double qualityScore = Double.NEGATIVE_INFINITY;
    public double timestampSeconds = 0.0;
    public double measurementAgeSeconds = 0.0;
    public double translationErrorMeters = 0.0;
    public double headingErrorDegrees = 0.0;
    public String statusMessage = "No measurement processed yet";
  }

  private static final String CAMERA_RIGHT = "limelight-right";
  private static final String CAMERA_LEFT = "limelight-left";

  // Smaller standard deviation means "trust this measurement more" in the pose
  // estimator. We keep the names explicit so students can connect the number with
  // the estimator behavior.
  private static final double LARGE_ROTATION_STD_DEV = 999999999.0;
  private static final double MIN_MT2_TAG_AREA = 0.05;
  private static final double MIN_MT1_TAG_AREA = 0.20;
  private static final double MAX_MT2_TAG_DISTANCE_METERS = 6.0;
  private static final double MAX_MT1_TAG_DISTANCE_METERS = 5.0;
  private static final double MAX_MT2_AMBIGUITY = 0.60;
  private static final double MAX_MT1_AMBIGUITY = 0.35;
  private static final double MAX_LATENCY_MILLISECONDS = 90.0;
  private static final double MAX_MEASUREMENT_AGE_SECONDS = 0.20;
  private static final double MAX_MT1_TRANSLATION_JUMP_METERS = 2.0;
  private static final double MAX_MT2_TRANSLATION_JUMP_METERS = 1.5;
  private static final double MAX_MT1_HEADING_JUMP_DEGREES = 50.0;
  private static final double CAMERA_SWITCH_QUALITY_MARGIN = 1.50;

  /** Creates a new Limelight pose subsystem. */

  // Making a static instance of the subsystem lets other classes share the latest
  // accepted vision pose without building duplicate Limelight readers.
  private static Limelight_Pose _instance;

  /**
   * Returns the shared Limelight pose subsystem instance.
   *
   * We use a single instance so all code paths read the same camera decision state.
   */
  public static Limelight_Pose getInstance() {
    if (_instance == null) {
      _instance = new Limelight_Pose();
    }
    return _instance;
  }

  public boolean poseUpdatesFromCameraActive = true;

  public PoseEstimate poseCamEstimate;
  public double poseUpdateXYTrustFactor = 0;
  public double poseUpdateRotTrustFactor = LARGE_ROTATION_STD_DEV;
  public boolean poseUpdateAvailable = false;

  private boolean poseUpdateAvailableCam1 = false;
  private boolean poseUpdateAvailableCam2 = false;
  private boolean usingCam1MT1 = false;
  private boolean usingCam2MT1 = false;
  private boolean usingCam1MT2 = false;
  private boolean usingCam2MT2 = false;

  private double timestampCam1Previous = 0.0;
  private double timestampCam2Previous = 0.0;

  private Pose2d currentDrivePose = Pose2d.kZero;
  private boolean driveStateAvailable = false;
  public double currentDriveTheta;
  public double currentDriveYawRate;
  public double currentDriveLinearSpeedMetersPerSecond;
  private String preferredCameraName = "None";

  private CameraMeasurementDecision cam1Decision = new CameraMeasurementDecision();
  private CameraMeasurementDecision cam2Decision = new CameraMeasurementDecision();

  /** Creates the subsystem object that manages Limelight pose updates. */
  public Limelight_Pose() {
  }

  /**
   * Enables pose updates from the Limelights.
   *
   * This is useful when we want the robot to resume vision corrections after they
   * have been temporarily disabled for testing or debugging.
   */
  public void SetPoseCameraActive() {
    poseUpdatesFromCameraActive = true;
  }

  /**
   * Disables pose updates from the Limelights.
   *
   * When disabled, the drivetrain will rely only on onboard odometry and gyro
   * measurements until vision is turned back on.
   */
  public void ClearPoseCameraActive() {
    poseUpdatesFromCameraActive = false;
  }

  /**
   * Evaluates the right Limelight and stores whether it produced a fresh, useful
   * pose update this loop.
   *
   * The method reads both MegaTag1 and MegaTag2, rejects stale frames, chooses the
   * best pose mode for that frame, and remembers why the decision was made.
   */
  public void SetPoseEstimateInfoCam1() {
    cam1Decision = evaluateCameraMeasurement(CAMERA_RIGHT, timestampCam1Previous);
    if (cam1Decision.processedFreshFrame) {
      timestampCam1Previous = cam1Decision.timestampSeconds;
    }

    poseUpdateAvailableCam1 = cam1Decision.acceptedMeasurement;
    usingCam1MT1 = cam1Decision.usingMegaTag1;
    usingCam1MT2 = cam1Decision.usingMegaTag2;
  }

  /**
   * Evaluates the left Limelight and stores whether it produced a fresh, useful
   * pose update this loop.
   *
   * The method mirrors the right-camera logic so both cameras are judged with the
   * same rules and can be compared fairly.
   */
  public void SetPoseEstimateInfoCam2() {
    cam2Decision = evaluateCameraMeasurement(CAMERA_LEFT, timestampCam2Previous);
    if (cam2Decision.processedFreshFrame) {
      timestampCam2Previous = cam2Decision.timestampSeconds;
    }

    poseUpdateAvailableCam2 = cam2Decision.acceptedMeasurement;
    usingCam2MT1 = cam2Decision.usingMegaTag1;
    usingCam2MT2 = cam2Decision.usingMegaTag2;
  }

  /**
   * Chooses the best accepted camera measurement for the drivetrain to fuse.
   *
   * We compare both cameras in the same loop and forward only the strongest fresh
   * measurement. This prevents "first camera wins" behavior and makes better use of
   * having two Limelights mounted on the robot.
   */
  public void SetPoseEstimateForDrive() {
    poseUpdateAvailable = false;
    poseCamEstimate = null;
    poseUpdateXYTrustFactor = 0.0;
    poseUpdateRotTrustFactor = LARGE_ROTATION_STD_DEV;

    CameraMeasurementDecision selectedDecision = choosePreferredDecision();

    if (selectedDecision != null) {
      poseCamEstimate = selectedDecision.selectedEstimate;
      poseUpdateXYTrustFactor = selectedDecision.xyStdDev;
      poseUpdateRotTrustFactor = selectedDecision.thetaStdDev;
      poseUpdateAvailable = true;
      preferredCameraName = selectedDecision.cameraName;
    }
  }

  /**
   * Marks the currently selected vision correction as consumed.
   *
   * The drivetrain calls this after it fuses the selected measurement so we do not
   * accidentally apply the same camera frame again on a later loop.
   */
  public void UpdateVisionCorrectionAdded() {
    poseUpdateAvailable = false;
    poseUpdateAvailableCam1 = false;
    poseUpdateAvailableCam2 = false;
  }

  /**
   * Stores the drivetrain pose and motion state used to validate vision updates.
   *
   * MegaTag2 expects fresh robot orientation information, and our reliability
   * checks also compare incoming vision poses against the drivetrain's current
   * estimate. Passing the full drive state in here once per loop lets us reject
   * late or unreasonable camera frames more safely.
   */
  public void CollectDriveState(Pose2d drivePose, double driveYawRate, double driveLinearSpeedMetersPerSecond) {
    currentDrivePose = drivePose;
    driveStateAvailable = true;
    currentDriveTheta = drivePose.getRotation().getDegrees();
    currentDriveYawRate = driveYawRate;
    currentDriveLinearSpeedMetersPerSecond = driveLinearSpeedMetersPerSecond;
  }

  /**
   * Pushes the latest drivetrain orientation into both Limelights.
   *
   * MegaTag2 uses the robot's heading and turn rate as an input, so we update both
   * cameras before asking them for new pose estimates.
   */
  private void updateMegaTag2Orientation() {
    LimelightHelpers.SetRobotOrientation(CAMERA_RIGHT, currentDriveTheta, currentDriveYawRate, 0, 0, 0, 0);
    LimelightHelpers.SetIMUMode(CAMERA_RIGHT, 0);
    LimelightHelpers.SetRobotOrientation(CAMERA_LEFT, currentDriveTheta, currentDriveYawRate, 0, 0, 0, 0);
    LimelightHelpers.SetIMUMode(CAMERA_LEFT, 0);
  }

  /**
   * Reads both pose modes from one camera and decides whether this loop produced a
   * usable new measurement.
   *
   * The method rejects stale frames first, then prefers MegaTag1 when tag geometry
   * is excellent, otherwise falls back to MegaTag2 for translation-only help.
   */
  private CameraMeasurementDecision evaluateCameraMeasurement(String cameraName, double previousTimestamp) {
    CameraMeasurementDecision decision = new CameraMeasurementDecision();
    decision.cameraName = cameraName;

    try {
      PoseEstimate megaTag1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
      PoseEstimate megaTag2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
      PoseEstimate freshestEstimate = getFreshestEstimate(megaTag1Estimate, megaTag2Estimate);
      boolean megaTag1Fresh = isEstimateFresh(megaTag1Estimate, previousTimestamp);
      boolean megaTag2Fresh = isEstimateFresh(megaTag2Estimate, previousTimestamp);

      if (freshestEstimate == null) {
        decision.statusMessage = "No pose data available from the camera";
        return decision;
      }

      decision.timestampSeconds = freshestEstimate.timestampSeconds;
      decision.measurementAgeSeconds = Math.max(0.0, Timer.getFPGATimestamp() - freshestEstimate.timestampSeconds);

      if (freshestEstimate.tagCount < 1) {
        fillRejectedFrameDetails(decision, freshestEstimate);
        decision.statusMessage = "Camera sees no AprilTags";
        return decision;
      }

      if (freshestEstimate.timestampSeconds <= previousTimestamp) {
        fillRejectedFrameDetails(decision, freshestEstimate);
        decision.statusMessage = "Camera frame is stale and was already processed";
        return decision;
      }

      decision.processedFreshFrame = true;

      if (freshestEstimate.latency > MAX_LATENCY_MILLISECONDS) {
        fillRejectedFrameDetails(decision, freshestEstimate);
        decision.statusMessage = "Rejected fresh frame because latency was too high";
        return decision;
      }

      if (decision.measurementAgeSeconds > MAX_MEASUREMENT_AGE_SECONDS) {
        fillRejectedFrameDetails(decision, freshestEstimate);
        decision.statusMessage = "Rejected fresh frame because it was too old";
        return decision;
      }

      // Check freshness per pose mode instead of only once per camera. This keeps
      // an older MegaTag1 or MegaTag2 result from being reused just because the
      // other mode happened to update more recently.
      if (megaTag1Fresh
          && isMegaTag1Reliable(megaTag1Estimate)
          && passesDriveStateValidation(megaTag1Estimate, true, decision)) {
        fillDecisionFromEstimate(decision, megaTag1Estimate, true);
        decision.statusMessage = "Accepted fresh MegaTag1 frame";
      } else if (megaTag2Fresh
          && isMegaTag2Reliable(megaTag2Estimate)
          && passesDriveStateValidation(megaTag2Estimate, false, decision)) {
        fillDecisionFromEstimate(decision, megaTag2Estimate, false);
        decision.statusMessage = "Accepted fresh MegaTag2 translation update";
      } else {
        fillRejectedFrameDetails(decision, freshestEstimate);
        if (decision.statusMessage.equals("No measurement processed yet")) {
          decision.statusMessage = "Rejected fresh frame because tag geometry was too weak";
        }
      }
    } catch (Exception e) {
      decision.statusMessage = "Failed to read Limelight data: " + e.getMessage();
      DriverStation.reportError("Vision camera data not present for " + cameraName + ": " + e.getMessage(),
          e.getStackTrace());
    }

    return decision;
  }

  /**
   * Returns the newest pose estimate between MegaTag1 and MegaTag2.
   *
   * This lets us process each camera frame once, even if one pose mode updates a
   * tiny bit earlier than the other.
   */
  private PoseEstimate getFreshestEstimate(PoseEstimate megaTag1Estimate, PoseEstimate megaTag2Estimate) {
    if (megaTag1Estimate == null) {
      return megaTag2Estimate;
    }

    if (megaTag2Estimate == null) {
      return megaTag1Estimate;
    }

    if (megaTag2Estimate.timestampSeconds >= megaTag1Estimate.timestampSeconds) {
      return megaTag2Estimate;
    }

    return megaTag1Estimate;
  }

  /**
   * Returns whether a specific pose mode produced a new estimate this loop.
   *
   * We track freshness per pose mode because MegaTag1 and MegaTag2 may not update
   * at exactly the same time. A newer result in one mode should not make an older
   * result in the other mode look fresh by accident.
   */
  private boolean isEstimateFresh(PoseEstimate estimate, double previousTimestamp) {
    return estimate != null && estimate.timestampSeconds > previousTimestamp;
  }

  /**
   * Chooses which accepted camera update should be fused this loop.
   *
   * If both cameras look good, we use a small hysteresis margin before switching
   * away from the previously preferred camera. This helps prevent the robot from
   * bouncing between left and right camera updates when they are nearly equal.
   */
  private CameraMeasurementDecision choosePreferredDecision() {
    if (!poseUpdateAvailableCam1 && !poseUpdateAvailableCam2) {
      return null;
    }

    if (poseUpdateAvailableCam1 && !poseUpdateAvailableCam2) {
      return cam1Decision;
    }

    if (!poseUpdateAvailableCam1 && poseUpdateAvailableCam2) {
      return cam2Decision;
    }

    if (CAMERA_RIGHT.equals(preferredCameraName)
        && cam2Decision.qualityScore <= cam1Decision.qualityScore + CAMERA_SWITCH_QUALITY_MARGIN) {
      return cam1Decision;
    }

    if (CAMERA_LEFT.equals(preferredCameraName)
        && cam1Decision.qualityScore <= cam2Decision.qualityScore + CAMERA_SWITCH_QUALITY_MARGIN) {
      return cam2Decision;
    }

    if (cam2Decision.qualityScore > cam1Decision.qualityScore) {
      return cam2Decision;
    }

    return cam1Decision;
  }

  /**
   * Checks whether MegaTag1 is strong enough to trust for both translation and
   * rotation.
   *
   * MegaTag1 is most helpful when we can see multiple tags with large image area,
   * low ambiguity, and reasonable distance. Those conditions usually mean the robot
   * has a solid geometric solution.
   */
  private boolean isMegaTag1Reliable(PoseEstimate estimate) {
    if (estimate == null) {
      return false;
    }

    if (estimate.tagCount < 2) {
      return false;
    }

    if (estimate.avgTagArea < MIN_MT1_TAG_AREA) {
      return false;
    }

    if (estimate.avgTagDist > MAX_MT1_TAG_DISTANCE_METERS) {
      return false;
    }

    return getMaxAmbiguity(estimate) <= MAX_MT1_AMBIGUITY;
  }

  /**
   * Checks whether MegaTag2 is strong enough to trust for translation help.
   *
   * MegaTag2 can still improve X/Y position even when we do not want to trust its
   * heading, so the requirements are a little more forgiving than MegaTag1.
   */
  private boolean isMegaTag2Reliable(PoseEstimate estimate) {
    if (estimate == null) {
      return false;
    }

    if (estimate.tagCount < 1) {
      return false;
    }

    if (estimate.avgTagArea < MIN_MT2_TAG_AREA) {
      return false;
    }

    if (estimate.avgTagDist > MAX_MT2_TAG_DISTANCE_METERS) {
      return false;
    }

    return getMaxAmbiguity(estimate) <= MAX_MT2_AMBIGUITY;
  }

  /**
   * Copies an accepted pose estimate into the camera decision object.
   *
   * This method stores the selected pose, computes estimator standard deviations,
   * and assigns a quality score that can be compared against the other camera.
   */
  private void fillDecisionFromEstimate(CameraMeasurementDecision decision, PoseEstimate estimate,
      boolean usingMegaTag1) {
    updateDecisionMetrics(decision, estimate);
    decision.acceptedMeasurement = true;
    decision.usingMegaTag1 = usingMegaTag1;
    decision.usingMegaTag2 = !usingMegaTag1;
    decision.selectedEstimate = estimate;
    decision.tagCount = estimate.tagCount;
    decision.avgTagArea = estimate.avgTagArea;
    decision.avgTagDist = estimate.avgTagDist;
    decision.tagSpan = estimate.tagSpan;
    decision.maxAmbiguity = getMaxAmbiguity(estimate);
    decision.xyStdDev = calculateXYStdDev(estimate, usingMegaTag1);
    decision.thetaStdDev = calculateThetaStdDev(estimate, usingMegaTag1);
    decision.qualityScore = calculateQualityScore(estimate, usingMegaTag1);
  }

  /**
   * Stores basic details for a rejected frame.
   *
   * We still publish these values so students can see what the camera observed and
   * understand why the robot ignored that frame.
   */
  private void fillRejectedFrameDetails(CameraMeasurementDecision decision, PoseEstimate estimate) {
    updateDecisionMetrics(decision, estimate);
    decision.selectedEstimate = estimate;
    decision.qualityScore = calculateQualityScore(estimate, false);
  }

  /**
   * Updates the diagnostic values we publish for a camera frame.
   *
   * This includes what the camera saw plus how far that pose is from the
   * drivetrain's current estimate. Those extra values make it much easier to tune
   * rejection thresholds on the practice field.
   */
  private void updateDecisionMetrics(CameraMeasurementDecision decision, PoseEstimate estimate) {
    decision.selectedEstimate = estimate;
    decision.timestampSeconds = estimate.timestampSeconds;
    decision.measurementAgeSeconds = Math.max(0.0, Timer.getFPGATimestamp() - estimate.timestampSeconds);
    decision.tagCount = estimate.tagCount;
    decision.avgTagArea = estimate.avgTagArea;
    decision.avgTagDist = estimate.avgTagDist;
    decision.tagSpan = estimate.tagSpan;
    decision.maxAmbiguity = getMaxAmbiguity(estimate);

    if (driveStateAvailable) {
      decision.translationErrorMeters = estimate.pose.getTranslation().getDistance(currentDrivePose.getTranslation());
      decision.headingErrorDegrees = Math.abs(
          estimate.pose.getRotation().minus(currentDrivePose.getRotation()).getDegrees());
    }
  }

  /**
   * Checks whether a camera frame is reasonably close to the drivetrain estimate.
   *
   * This protects the robot from one bad AprilTag solve causing a huge pose jump.
   * MegaTag1 must agree in both translation and heading, while MegaTag2 only needs
   * to agree in translation because we already treat its heading as untrusted.
   */
  private boolean passesDriveStateValidation(PoseEstimate estimate, boolean usingMegaTag1,
      CameraMeasurementDecision decision) {
    updateDecisionMetrics(decision, estimate);

    if (!driveStateAvailable) {
      return true;
    }

    double maxTranslationJumpMeters = usingMegaTag1 ? MAX_MT1_TRANSLATION_JUMP_METERS : MAX_MT2_TRANSLATION_JUMP_METERS;
    if (decision.translationErrorMeters > maxTranslationJumpMeters) {
      decision.statusMessage = "Rejected fresh frame because pose jump was too large";
      return false;
    }

    if (usingMegaTag1 && decision.headingErrorDegrees > MAX_MT1_HEADING_JUMP_DEGREES) {
      decision.statusMessage = "Rejected fresh frame because heading jump was too large";
      return false;
    }

    return true;
  }

  /**
   * Calculates how much we should trust a vision update's X and Y position.
   *
   * Lower values mean the estimator will trust the camera more. We reward multiple
   * tags, larger tag area, and shorter distance, and we become more conservative
   * when ambiguity grows.
   */
  private double calculateXYStdDev(PoseEstimate estimate, boolean usingMegaTag1) {
    double xyStdDev = 1.20;

    if (estimate.tagCount >= 2) {
      xyStdDev -= 0.25;
    }

    if (estimate.avgTagArea >= 0.20) {
      xyStdDev -= 0.20;
    }

    if (estimate.avgTagDist > 3.0) {
      xyStdDev += 0.25;
    }

    if (estimate.avgTagDist > 4.5) {
      xyStdDev += 0.40;
    }

    if (getMaxAmbiguity(estimate) > 0.25) {
      xyStdDev += 0.25;
    }

    if (!usingMegaTag1) {
      // MegaTag2 translation is useful, but we stay a little more conservative than
      // a strong MegaTag1 solution.
      xyStdDev += 0.15;
    }

    // Fast motion makes camera measurements less reliable, so we automatically
    // trust them a little less when the drivetrain is moving aggressively.
    xyStdDev += Math.abs(currentDriveYawRate) * 0.003;
    xyStdDev += Math.abs(currentDriveLinearSpeedMetersPerSecond) * 0.12;

    return clamp(xyStdDev, 0.35, 2.50);
  }

  /**
   * Calculates how much we should trust a vision update's rotation.
   *
   * MegaTag2 is treated as translation-only here, so we set a huge rotational
   * standard deviation and let the gyro remain the primary source of heading.
   */
  private double calculateThetaStdDev(PoseEstimate estimate, boolean usingMegaTag1) {
    if (!usingMegaTag1) {
      return LARGE_ROTATION_STD_DEV;
    }

    double thetaStdDev = 0.50;

    if (estimate.tagCount >= 3) {
      thetaStdDev -= 0.10;
    }

    if (estimate.avgTagArea < 0.30) {
      thetaStdDev += 0.15;
    }

    if (getMaxAmbiguity(estimate) > 0.20) {
      thetaStdDev += 0.20;
    }

    // When the robot is spinning quickly, it is safer to lean more heavily on the
    // gyro for heading and be less aggressive with vision rotation corrections.
    thetaStdDev += Math.abs(currentDriveYawRate) * 0.002;

    return clamp(thetaStdDev, 0.35, 1.50);
  }

  /**
   * Produces a simple quality score so two accepted camera updates can be compared.
   *
   * Higher scores mean the camera saw more useful tag information. We slightly
   * prefer MegaTag1 when all else is equal because it can contribute both
   * translation and rotation.
   */
  private double calculateQualityScore(PoseEstimate estimate, boolean usingMegaTag1) {
    double score = 0.0;
    double measurementAgeSeconds = Math.max(0.0, Timer.getFPGATimestamp() - estimate.timestampSeconds);

    score += estimate.tagCount * 3.0;
    score += Math.min(estimate.avgTagArea, 1.0) * 4.0;
    score += Math.max(0.0, 4.0 - estimate.avgTagDist);
    score += Math.max(0.0, estimate.tagSpan);
    score -= getMaxAmbiguity(estimate) * 5.0;
    score -= measurementAgeSeconds * 10.0;
    score -= Math.abs(currentDriveYawRate) * 0.01;

    if (!usingMegaTag1) {
      score -= 0.5;
    }

    return score;
  }

  /**
   * Finds the worst ambiguity value in a pose estimate's tag list.
   *
   * Ambiguity tells us how unsure the camera is about a tag solution, so the worst
   * ambiguity in the frame is a useful quick summary for trust decisions.
   */
  private double getMaxAmbiguity(PoseEstimate estimate) {
    if (estimate == null || estimate.rawFiducials == null || estimate.rawFiducials.length == 0) {
      return 0.0;
    }

    double maxAmbiguity = 0.0;

    for (RawFiducial fiducial : estimate.rawFiducials) {
      if (fiducial.ambiguity > maxAmbiguity) {
        maxAmbiguity = fiducial.ambiguity;
      }
    }

    return maxAmbiguity;
  }

  /**
   * Limits a value to a safe range.
   *
   * We clamp our estimator standard deviations so a strong frame does not become
   * unrealistically trusted and a weak frame does not become effectively useless.
   */
  private double clamp(double value, double minValue, double maxValue) {
    return Math.max(minValue, Math.min(maxValue, value));
  }

  /**
   * Publishes detailed vision debugging information to SmartDashboard.
   *
   * The goal is to make the robot's decisions understandable: which camera saw a
   * frame, which pose mode was chosen, what the tag quality looked like, and why a
   * frame was accepted or rejected.
   */
  private void updateShuffleboard() {
    publishCameraTelemetry("Vision/Right", cam1Decision);
    publishCameraTelemetry("Vision/Left", cam2Decision);

    SmartDashboard.putBoolean("Vision/Selected/Available", poseUpdateAvailable);
    SmartDashboard.putString("Vision/Selected/Camera", getSelectedCameraLabel());
    SmartDashboard.putString("Vision/Selected/PreferredCamera", preferredCameraName);
    SmartDashboard.putNumber("Vision/Selected/XYStdDev", poseUpdateXYTrustFactor);
    SmartDashboard.putNumber("Vision/Selected/ThetaStdDev", poseUpdateRotTrustFactor);
    SmartDashboard.putNumber("Vision/Drive/YawRateDegPerSec", currentDriveYawRate);
    SmartDashboard.putNumber("Vision/Drive/LinearSpeedMetersPerSecond", currentDriveLinearSpeedMetersPerSecond);

    SmartDashboard.putBoolean("Vision/Right/UsingMT1", usingCam1MT1);
    SmartDashboard.putBoolean("Vision/Right/UsingMT2", usingCam1MT2);
    SmartDashboard.putBoolean("Vision/Left/UsingMT1", usingCam2MT1);
    SmartDashboard.putBoolean("Vision/Left/UsingMT2", usingCam2MT2);
  }

  /**
   * Publishes one camera's decision details under a shared dashboard prefix.
   *
   * Using separate left and right keys prevents one camera from overwriting the
   * other camera's telemetry while we tune the system.
   */
  private void publishCameraTelemetry(String dashboardPrefix, CameraMeasurementDecision decision) {
    SmartDashboard.putBoolean(dashboardPrefix + "/FreshFrame", decision.processedFreshFrame);
    SmartDashboard.putBoolean(dashboardPrefix + "/Accepted", decision.acceptedMeasurement);
    SmartDashboard.putString(dashboardPrefix + "/Mode", getModeLabel(decision));
    SmartDashboard.putString(dashboardPrefix + "/Status", decision.statusMessage);
    SmartDashboard.putNumber(dashboardPrefix + "/Timestamp", decision.timestampSeconds);
    SmartDashboard.putNumber(dashboardPrefix + "/MeasurementAgeSeconds", decision.measurementAgeSeconds);
    SmartDashboard.putNumber(dashboardPrefix + "/TagCount", decision.tagCount);
    SmartDashboard.putNumber(dashboardPrefix + "/AvgTagArea", decision.avgTagArea);
    SmartDashboard.putNumber(dashboardPrefix + "/AvgTagDist", decision.avgTagDist);
    SmartDashboard.putNumber(dashboardPrefix + "/TagSpan", decision.tagSpan);
    SmartDashboard.putNumber(dashboardPrefix + "/MaxAmbiguity", decision.maxAmbiguity);
    SmartDashboard.putNumber(dashboardPrefix + "/TranslationErrorMeters", decision.translationErrorMeters);
    SmartDashboard.putNumber(dashboardPrefix + "/HeadingErrorDegrees", decision.headingErrorDegrees);
    SmartDashboard.putNumber(dashboardPrefix + "/QualityScore", decision.qualityScore);
    SmartDashboard.putNumber(dashboardPrefix + "/XYStdDev", decision.xyStdDev);
    SmartDashboard.putNumber(dashboardPrefix + "/ThetaStdDev", decision.thetaStdDev);
  }

  /**
   * Returns a short label describing which camera measurement was chosen.
   *
   * This helps students quickly see whether the drivetrain is currently trusting the
   * right camera, the left camera, or no vision update at all.
   */
  private String getSelectedCameraLabel() {
    if (!poseUpdateAvailable || poseCamEstimate == null) {
      return "None";
    }

    if (cam1Decision.acceptedMeasurement && cam1Decision.selectedEstimate == poseCamEstimate) {
      return "Right";
    }

    if (cam2Decision.acceptedMeasurement && cam2Decision.selectedEstimate == poseCamEstimate) {
      return "Left";
    }

    return "Unknown";
  }

  /**
   * Returns a short label describing which Limelight pose mode was used.
   *
   * We keep this text simple so the dashboard is easy to read during a match or a
   * practice session.
   */
  private String getModeLabel(CameraMeasurementDecision decision) {
    if (decision.usingMegaTag1) {
      return "MegaTag1";
    }

    if (decision.usingMegaTag2) {
      return "MegaTag2";
    }

    return "None";
  }

  @Override
  public void periodic() {
    // This method runs every scheduler loop and decides whether a brand new
    // Limelight frame should be forwarded to the drivetrain's pose estimator.
    if (poseUpdatesFromCameraActive) {
      updateMegaTag2Orientation();
      SetPoseEstimateInfoCam1();
      SetPoseEstimateInfoCam2();
      SetPoseEstimateForDrive();
    } else {
      poseUpdateAvailable = false;
      poseUpdateAvailableCam1 = false;
      poseUpdateAvailableCam2 = false;
      usingCam1MT1 = false;
      usingCam1MT2 = false;
      usingCam2MT1 = false;
      usingCam2MT2 = false;
      poseCamEstimate = null;
    }

    updateShuffleboard();
  }
}
