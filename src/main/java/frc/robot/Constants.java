// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.math.Convert;
import frc.robot.util.math.Vector2;
import java.util.HashMap;

public final class Constants {
  public static final boolean TUNING_MODE = true; // * TODO disable debug mode for competition

  public static final class Drivetrain {
    public static final int kRightFrontCanId = 0;
    public static final int kRightBackCanId = 1;
    public static final int kLeftFrontCanId = 2;
    public static final int kLeftBackCanId = 3;
  }

  public static final class Tower {
    public static final int kUpperMotorCanId = 7;
    public static final int kLowerMotorCanId = 5;

    public static final int kLowerMotorChannel = 16;
  }

  public static final class Turret {
    public static final int kMotorCanId = 9;
  }

  public static final class Shooter {
    public static final int kFlywheelMotorCanId = 10;
  }

  public static final class Intake {
    public static final int kMotorCanId = 8;

    public static final int kSolenoidUp = 15;
    public static final int kSolenoidDown = 0;
  }

  public static final class Climber {
    public static final int kLeftMotorCanId = 11;
    public static final int kRightMotorCanId = 12;

    public static final int kLeftMotorChannel = 19;
    public static final int kRightMotorChannel = 0;
  }

  public static final class Field {
    public static final double kVisionTargetHeight = 2.605; // meters from ground
    public static final double kGoalHeight = 2.630; // meters from ground
    public static final double kGoalRadius = 0.650; // meters

    public static final double kBallRadius = Convert.inchesToMeters(9.5) / 2.0; // meters
    public static final double kBallMass = 0.270; // kg
    public static final HashMap<Alliance, Color> kBallColors = new HashMap<>();

    static {
      kBallColors.put(Alliance.Blue, new Color(0.15, 0.41, 0.44));
      kBallColors.put(Alliance.Red, new Color(0.47, 0.37, 0.15));
    }
  }

  public static final class Limelight {
    public static final double kMountingAngle = 39.0; // degrees up from horizontal
    public static final double kMountingHeight = Convert.inchesToMeters(40.0); // meters from ground
    public static final Vector2 kShooterOffset =
        new Vector2(Convert.inchesToMeters(5.0), Convert.inchesToMeters(-5.5));

    public static final LimelightMountDirection kMountingDirection =
        LimelightMountDirection.kPortrait;

    public static final int kPipelineDriving = 0;
    public static final int kPipelineShooting = 1;

    public enum LimelightMountDirection {
      kLandscape,
      kPortrait;
    }
  }

  public static final class ColorSensors {
    public static final I2C.Port kUpperSensorPort = I2C.Port.kOnboard;
    public static final I2C.Port kLowerSensorPort = I2C.Port.kMXP;
  }
}
