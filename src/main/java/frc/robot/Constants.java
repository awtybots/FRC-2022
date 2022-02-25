// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import java.util.HashMap;

public final class Constants {
  public static final boolean TUNING_MODE = true; // TODO false for competition

  public static final class Drivetrain {
    public static final int kRightFront = 0;
    public static final int kRightBack = 1;
    public static final int kLeftFront = 2;
    public static final int kLeftBack = 3;

    public static TalonFXConfiguration motorConfig() {
      TalonFXConfiguration conf = new TalonFXConfiguration();
      conf.voltageCompSaturation = 12.0;
      conf.openloopRamp = 1.0;
      return conf;
    }
  }

  public static final class Tower {
    public static final int kUpperMotor = 7;
    public static final int kLowerMotor = 5;

    public static TalonSRXConfiguration motorConfig() {
      TalonSRXConfiguration conf = new TalonSRXConfiguration();
      return conf;
    }
  }

  public static final class Turret {
    public static final int kMotor = 9;

    public static TalonSRXConfiguration motorConfig() {
      TalonSRXConfiguration conf = new TalonSRXConfiguration();
      conf.voltageCompSaturation = 12.0;
      return conf;
    }
  }

  public static final class Shooter {
    public static final int kFlywheelMotor = -1;

    public static TalonFXConfiguration motorConfig() {
      TalonFXConfiguration conf = new TalonFXConfiguration();
      conf.voltageCompSaturation = 12.0;
      return conf;
    }
  }

  public static final class Field {
    public static final double kVisionTargetHeight = 2.605; // meters from ground
    public static final double kGoalHeight = 2.630; // meters from ground

    public static final HashMap<Alliance, Color> kBallColors = new HashMap<>();

    static {
      kBallColors.put(Alliance.Blue, new Color(0.0, 0.0, 1.0)); // TODO
      kBallColors.put(Alliance.Red, new Color(1.0, 0.0, 0.0)); // TODO
    }
  }

  public static final class Limelight {
    public static final double kMountingAngle = 40.0; // degrees up from horizontal
    public static final double kMountingHeight =
        0.7; // TODO find correct number - meters from ground

    public static final LimelightMountDirection kMountingDirection =
        LimelightMountDirection.kLandscape;

    public enum LimelightMountDirection {
      kLandscape,
      kPortrait;
    }
  }

  public static final class ColorSensors {
    public static final I2C.Port kUpperSensorPort = I2C.Port.kOnboard;
    public static final I2C.Port kLowerSensorPort = I2C.Port.kMXP;
  }

  public static final class Intake {
    public static final int kMotor = 8;
    public static final int kSolenoidUp = -1;
    public static final int kSolenoidDown = -1;

    public static TalonSRXConfiguration motorConfig() {
      TalonSRXConfiguration conf = new TalonSRXConfiguration();
      return conf;
    }
  }

  public static final class Climber {
    public static final int kLeftMotor = -1;
    public static final int kRightMotor = -1;

    public static TalonFXConfiguration motorConfig() {
      TalonFXConfiguration conf = new TalonFXConfiguration();
      return conf;
    }
  }
}
