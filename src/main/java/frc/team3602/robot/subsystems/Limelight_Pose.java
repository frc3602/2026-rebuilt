// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3602.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3602.robot.LimelightHelpers;

public class Limelight_Pose extends SubsystemBase{
  /** Creates a new Limelight. */

  // Making a static instance of the Subsystem so some paramters
  // can be passed along to other subsystems.
  private static Limelight_Pose _instance;
  public static Limelight_Pose getInstance(){
      if(_instance == null){_instance = new Limelight_Pose();}
      return _instance;
    }

  public boolean poseUpdatesFromCameraActive = true;
  
  public LimelightHelpers.PoseEstimate poseCamEstimate;
  private LimelightHelpers.PoseEstimate poseCam1Estimate;
  private LimelightHelpers.PoseEstimate poseCam2Estimate;
  private LimelightHelpers.PoseEstimate poseCam1MT1Estimate;
  private LimelightHelpers.PoseEstimate poseCam2MT1Estimate;
  private LimelightHelpers.PoseEstimate poseCam1MT2Estimate;
  private LimelightHelpers.PoseEstimate poseCam2MT2Estimate;

  private int tagsFoundAtPoseEstimateCam1 = 0;
  private int tagsFoundAtPoseEstimateCam2 = 0;
  private double tagAreaAtPoseEstimateCam1 = 0;
  private double tagAreaAtPoseEstimateCam2 = 0;
  public double poseUpdateXYTrustFactor = 0;
  private double poseUpdateXYTrustFactorCam1 = 0;
  private double poseUpdateXYTrustFactorCam2 = 0;
  public double poseUpdateRotTrustFactor = 999999999;
  private double poseUpdateRotTrustFactorCam1 = 999999999;
  private double poseUpdateRotTrustFactorCam2 = 999999999;
  public boolean poseUpdateAvailable = false;
  private boolean poseUpdateAvailableCam1 = false;
  private boolean poseUpdateAvailableCam2 = false;

  private boolean usingCam1MT1 = false;
  private boolean usingCam2MT1 = false;
  private boolean usingCam1MT2 = false;
  private boolean usingCam2MT2 = false;
  private double timestampCam1Previous = 0.0;
  private double timestampCam2Previous = 0.0;

  public double currentDriveTheta;
  // TODO(Codex-MT2): Store live drivetrain yaw rate so MegaTag2 gets fresh orientation data before each read.
  public double currentDriveYawRate;


  public Limelight_Pose(){}

  public void SetPoseCameraActive(){
    poseUpdatesFromCameraActive = true;
  }

  public void ClearPoseCameraActive(){
    poseUpdatesFromCameraActive = false;
  }

  public void SetPoseEstimateInfoCam1(){

    try{
    poseCam1MT1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
    poseCam1MT2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");

      // Collect all the vision update information if seeing a tag with a newer timestamp than previous iteration.
      if ((poseCam1MT2Estimate.tagCount > 0) && (poseCam1MT2Estimate.timestampSeconds > timestampCam1Previous)){

        // Begin by collecting the MT2 Pose Estimate (Will end up essentially ignoring the rotation value when adding to Pose Estimator)
        poseCam1Estimate = poseCam1MT2Estimate;

        // Save the timestamp to previous to prevent duplicate updating
        timestampCam1Previous = poseCam1MT2Estimate.timestampSeconds;

        // Collect miscellaneous info to help determine trust factor
        tagsFoundAtPoseEstimateCam1 = poseCam1MT2Estimate.tagCount;
        tagAreaAtPoseEstimateCam1 = poseCam1MT2Estimate.avgTagArea;

        //  starting with a high trust factor (small number) and adding to it as needed
        poseUpdateXYTrustFactorCam1 = 0.7;

        }




        // a whole bunch of code would go here if adjusting the trust factor based on various measurements




        // Using MT1 if conditions are ideal, multiple tags seen, and tag area greater than determined value
        if ((poseUpdateXYTrustFactorCam1 == 0.7) && (tagsFoundAtPoseEstimateCam1 > 1) 
          && (tagAreaAtPoseEstimateCam1 > 0.2)){
          usingCam1MT1 = true;
          usingCam1MT2 = false;
          poseCam1Estimate = poseCam1MT1Estimate;
          poseUpdateRotTrustFactorCam1 = 0.5;
          poseUpdateAvailableCam1 = true;
        }
        else{
          usingCam1MT2 = true;
          usingCam1MT1 = false;
          poseCam1Estimate = poseCam1MT2Estimate;
          poseUpdateRotTrustFactorCam1 = 999999999;
          poseUpdateAvailableCam1 = true;
          }
      
      // Only clear out pose available data if no tags were found
      if (poseCam1MT2Estimate.tagCount < 1){
        usingCam1MT1 = false;
        usingCam1MT2 = false;
        poseUpdateAvailableCam1 = false;
      }

    } catch (Exception e) {
        DriverStation.reportError("Camera 1 Data Not Present" + e.getMessage(), e.getStackTrace());
      }  
  }

    public void SetPoseEstimateInfoCam2(){

    try{
    poseCam2MT1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
    poseCam2MT2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");

      // Collect all the vision update information if seeing a tag with a newer timestamp than previous iteration.
      if ((poseCam2MT2Estimate.tagCount > 0) && (poseCam2MT2Estimate.timestampSeconds > timestampCam2Previous)){

        // Begin by collecting the MT2 Pose Estimate (Will end up essentially ignoring the rotation value when adding to Pose Estimator)
        poseCam2Estimate = poseCam2MT2Estimate;

        // Save the timestamp to previous to prevent duplicate updating
        timestampCam2Previous = poseCam2MT2Estimate.timestampSeconds;

        // Collect miscellaneous info to help determine trust factor
        tagsFoundAtPoseEstimateCam2 = poseCam2MT2Estimate.tagCount;
        tagAreaAtPoseEstimateCam2 = poseCam2MT2Estimate.avgTagArea;

        //  starting with a high trust factor (small number) and adding to it as needed
        poseUpdateXYTrustFactorCam2 = 0.7;

        }




        // a whole bunch of code would go here if adjusting the trust factor based on various measurements




        // Using MT1 if conditions are ideal, multiple tags seen, and tag area greater than determined value
        if ((poseUpdateXYTrustFactorCam2 == 0.7) && (tagsFoundAtPoseEstimateCam2 > 1) 
          && (tagAreaAtPoseEstimateCam2 > 0.2)){
          usingCam2MT1 = true;
          usingCam2MT2 = false;
          poseCam2Estimate = poseCam2MT1Estimate;
          poseUpdateRotTrustFactorCam2 = 0.5;
          poseUpdateAvailableCam2 = true;
        }
        else{
          usingCam2MT2 = true;
          usingCam2MT1 = false;
          poseCam2Estimate = poseCam2MT2Estimate;
          poseUpdateRotTrustFactorCam2 = 999999999;
          poseUpdateAvailableCam2 = true;
          }
      
      // Only clear out pose available data if no tags were found
      if (poseCam2MT2Estimate.tagCount < 1){
        usingCam2MT1 = false;
        usingCam2MT2 = false;
        poseUpdateAvailableCam2 = false;
      }

    } catch (Exception e) {
        DriverStation.reportError("Camera 1 Data Not Present" + e.getMessage(), e.getStackTrace());
      }  
  }


public void SetPoseEstimateForDrive(){

  // Camera 1
  if (!poseUpdateAvailable && poseUpdateAvailableCam1){

    // Only using MT1 estimates for now until we find out what is up with MT2
    if (usingCam1MT1){

      poseCamEstimate = poseCam1Estimate;
      poseUpdateXYTrustFactor = poseUpdateXYTrustFactorCam1;
      poseUpdateRotTrustFactor = poseUpdateRotTrustFactorCam1;

      poseUpdateAvailable = true;
    }
    else{
      poseUpdateAvailable = false;
    }
  }

  // Camera 2
  if (!poseUpdateAvailable && poseUpdateAvailableCam2){

    if (usingCam2MT1){

      poseCamEstimate = poseCam2Estimate;
      poseUpdateXYTrustFactor = poseUpdateXYTrustFactorCam2;
      poseUpdateRotTrustFactor = poseUpdateRotTrustFactorCam2;

      poseUpdateAvailable = true;
    }
    else{
      poseUpdateAvailable = false;
    }
  }
}

  
  
  // This method clears out the pose update available flag to help prevent multiple offsets of same data.
  // This method also checks if we're in a "Was Tipped" state and using super aggressive correction.  If so
  // we monitor how many vision corrections have taken place, and clear the "Was Tipped" state accordingly.
  public void UpdateVisionCorrectionAdded(){

    // Clear out the "Pose Update Available" flags
    poseUpdateAvailable = false; 
    poseUpdateAvailableCam1 = false;
    poseUpdateAvailableCam2 = false;

}

// TODO(Codex-MT2): Collect both yaw and yaw rate from the drivetrain for MegaTag2 orientation updates.
public void CollectDriveThetaValue(double driveTheta, double driveYawRate){
  currentDriveTheta = driveTheta;
  currentDriveYawRate = driveYawRate;
}

  // TODO(Codex-MT2): Push fresh robot orientation to both Limelights before reading MegaTag2 pose estimates.
  private void updateMegaTag2Orientation() {
    LimelightHelpers.SetRobotOrientation("limelight-right", currentDriveTheta, currentDriveYawRate, 0, 0, 0, 0);
    LimelightHelpers.SetIMUMode("limelight-right", 0);
    LimelightHelpers.SetRobotOrientation("limelight-left", currentDriveTheta, currentDriveYawRate, 0, 0, 0, 0);
    LimelightHelpers.SetIMUMode("limelight-left", 0);
}

  private void updateShuffleboard() {

    SmartDashboard.putBoolean("Using MT1", usingCam1MT1);
    SmartDashboard.putBoolean("Using MT2", usingCam1MT2);

    SmartDashboard.putBoolean("Using MT1", usingCam2MT1);
    SmartDashboard.putBoolean("Using MT2", usingCam2MT2);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // // Sending the robot's current angle to the camera all the time
    // LimelightHelpers.SetRobotOrientation("limelight-right", currentDriveTheta, 0, 0, 0, 0, 0);
    // LimelightHelpers.SetIMUMode("limelight-right", 0);
    // LimelightHelpers.SetRobotOrientation("limelight-left", currentDriveTheta, 0, 0, 0, 0, 0);
    // LimelightHelpers.SetIMUMode("limelight-left", 0);

    // Looking for pose updates if activated
if (poseUpdatesFromCameraActive){
  // TODO(Codex-MT2): Update Limelight orientation first so MegaTag2 uses the latest drivetrain heading input.
  updateMegaTag2Orientation();
  SetPoseEstimateInfoCam1();
  SetPoseEstimateInfoCam2();
  SetPoseEstimateForDrive();
}
else {
  poseUpdateAvailable = false;

  poseUpdateAvailableCam1 = false;
  poseUpdateAvailableCam2 = false;

  usingCam1MT1 = false;
  usingCam1MT2 = false;

  usingCam2MT1 = false;
  usingCam2MT2 = false;
}

    updateShuffleboard();
  }  

}
