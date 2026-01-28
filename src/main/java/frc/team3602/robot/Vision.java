package frc.team3602.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team3602.robot.LimelightHelpers;
import frc.team3602.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.team3602.robot.LimelightHelpers.RawFiducial;


public class Vision {
    

    public double getDistToCamera(int id) {
        var tags =  LimelightHelpers.getRawFiducials("limelight-primary");
         return  0;
    }
    
    public double getTX() {
        return LimelightHelpers.getTX("limelight-primary");
    }

    public double getTurretTX() {
        return LimelightHelpers.getTX("limelight-turret");
    }

    public boolean getHasTarget() {
        return LimelightHelpers.getTV("limelight-primary");
    }

        public boolean getTurretHasTarget() {
        return LimelightHelpers.getTV("limelight-turret");
    }

    public double getTY() {
        return LimelightHelpers.getTY("limelight-primary");
    }

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-primary");
    NetworkTableEntry ty = table.getEntry("ty");

    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0; 

    // distance from the target to the floor
    double goalHeightInches = 60.0; 

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    //calculated Distance
    
//    public double getTargetDistance(
//         cameraAngle = 61.964,
//         cameraHeight = 0,
//         targetAngle = 90,
//         targetHeight = 0;
// ) Double {
//     return (targetHeight - cameraHeight) / Math.tan(Math.toRadians(cameraAngle + targetAngle))
// }

public double getDist() {
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }


    public double getPoseY() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-primary").pose.getY();
    }

    










}


