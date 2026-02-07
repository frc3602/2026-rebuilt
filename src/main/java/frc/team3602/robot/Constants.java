/*
 * Copyright (C) 2026 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

public final class Constants {
    public final class ShooterConstants {
        //Motor ID
        public final static int kShooterMotor1ID = 5;
        public final static int kShooterMotor2ID = 6;

        //Motor Speeds
        public final static double kShooterSpeed = .75;
    }

    public final class IntakeConstants {
        //Motor ID
        public final static int kIntakeMotorID = 8;
        
        //Motor Speeds
        public final static double kIntakeMotorSpeed = .5;

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

        public final static double kSpindexerMotorSpeed = -.3;

        public final static double kRecieveFuelSpeed = -0.4;
    }

    
}