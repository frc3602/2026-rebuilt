/*
 * Copyright (C) 2026 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public final class ShooterConstants {
        //Motor ID
        public final static int kShooterMotor1ID = 5;
        public final static int kShooterMotor2ID = 6;

        //Motor Speeds
        public final static double kShooterSpeed = .75;

        //Failsafe Speed
        public final static double kShooterFailsafeSpeed = -41.5;
    }

    public final class ClimberConstants {
        // Feature flag
        // Set this to true when the climber hardware is back on the robot and the
        // CAN ID below is correct. Leaving it false keeps the climber code in the
        // project without letting the robot talk to hardware that is not installed.
        public final static boolean kClimberEnabled = false;

        //Motor ID
        public final static int kClimberMotorID = 15;
    }
    public final class IntakeConstants {
        //Motor ID
        public final static int kIntakeMotorID = 8;
        public final static int kIntakePivotID = 13;
        public final static int kIntakePivotFollowID = 14;
        
        //Motor Speeds
        public final static double kIntakeMotorSpeed = -0.5;
        public final static double kPivotCurrentLimit = 10;

    }

    public final class TurretConstants {
        //Motor ID
        public final static int kTurretMotorID = 9;
        public final static int kTurretEncoderID = 10;

        //Motor Speeds
        public final static double kTurretMotorSpeed = .5;
    }

    public final class spindexerConstants {
        //Motor ID
        public final static int kSpindexerMotorID = 12;
        public final static int kReceiveMotorID = 11;

        // Feed scaling model:
        // shooter is fastest, receiver is slower than shooter, and spindexer is
        // slower than receiver.
        //
        // These are ratios (unitless), so 0.65 means "65% of the upstream speed".
        public final static double kReceiverToShooterSpeedRatio = 0.65;
        public final static double kSpindexerToReceiverSpeedRatio = 0.80;

        public final static double kSpindexerMotorSpeed = -.65;

        public final static double kRecieveFuelSpeed = .75;//.65
    }

    public final class FieldConstants {
        // Official REBUILT field coordinate system:
        // - (0, 0) is the blue-right corner when viewed from above
        // - +X points toward the red alliance wall
        // - +Y points left across the field
        //
        // These tower positions are the team's measured field targets in meters.
        // Keeping the measured numbers here lets turret aiming, distance
        // calculations, and autonomous tower shots all use the same real-world
        // reference points.
        public final static Translation2d kBlueTowerPosition = new Translation2d(4.634, 4.029);
        public final static Translation2d kRedTowerPosition = new Translation2d(11.919, 4.029);
    }

    
}
