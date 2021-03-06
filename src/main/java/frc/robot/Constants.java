// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import frc.util.math.Convert;
import frc.util.math.ShotMap;
import frc.util.math.Vector2;

public final class Constants {
    public static final boolean TUNING_MODE = false;

    public static final class Drivetrain {
        public static final int kRightFrontCanId = 0;
        public static final int kRightBackCanId = 1;
        public static final int kLeftFrontCanId = 2;
        public static final int kLeftBackCanId = 3;
    }

    public static final class Tower {
        public static final int kUpperMotorCanId = 7;
        public static final int kLowerMotorCanId = 8;

        public static final int kSolenoidUp = 10;
        public static final int kSolenoidDown = 8;
    }

    public static final class Turret {
        public static final int kMotorCanId = 9;
    }

    public static final class Shooter {
        public static final int kMotor1CanId = 10;
        public static final int kMotor2CanId = 13;
        public static final ShotMap shotMap = new ShotMap();

        static { // distance (meters), RPM
            shotMap.addShot(3.90, 1600);
            shotMap.addShot(4.20, 1630);
            shotMap.addShot(5.80, 2000);
            shotMap.addShot(6.37, 2200);
        }
    }

    public static final class Climber {
        public static final int kLeftMotorCanId = 11;
        public static final int kRightMotorCanId = 12;

        public static final int kTraverseSolenoidF = 0;
        public static final int kTraverseSolenoidR = 15;

        // public static final int kLeftMotorChannel = 19;
        // public static final int kRightMotorChannel = 0;
    }

    public static final class Field {
        public static final double kVisionTargetHeight = 2.605; // meters from ground
        public static final double kGoalHeight = 2.630; // meters from ground
        public static final double kGoalRadius = 0.650; // meters

        public static final double kBallRadius = Convert.inchesToMeters(9.5) / 2.0; // meters
        public static final double kBallMass = 0.270; // kg
    }

    public static final class Camera {
        public static final double kMountingAngle = 39.0; // degrees up from horizontal
        public static final double kMountingHeight =
                Convert.inchesToMeters(40.0); // meters from ground
        public static final Vector2 kShooterOffset =
                new Vector2(Convert.inchesToMeters(5.0), Convert.inchesToMeters(-5.5));

        public static final LimelightOrientation kOrientation = LimelightOrientation.kUpsideDown;

        public static final int kPipelineDriving = 0;
        public static final int kPipelineShooting = 1;

        public enum LimelightOrientation {
            kLandscape,
            kUpsideDown,
            kPortrait;
        }
    }

    public static final class ColorSensors {
        public static final I2C.Port kUpperSensorPort = I2C.Port.kOnboard;
        public static final I2C.Port kLowerSensorPort = I2C.Port.kMXP;
    }
}
