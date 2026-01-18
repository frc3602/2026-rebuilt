/*
 * Copyright (C) 2026 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import frc.team3602.robot.LimelightHelpers;
import frc.team3602.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.team3602.robot.LimelightHelpers.RawFiducial;

public class Vision {

    public double getDistToCamera(int id) {
        var tags = LimelightHelpers.getRawFiducials("limelight-primary");
        return 0;
    }

    public double getTX() {
        return LimelightHelpers.getTX("limelight-primary");
    }
}
