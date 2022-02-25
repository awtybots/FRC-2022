// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean DEBUG_MODE = true;

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
    public static final double visionTargetHeight = 2.605; // meters from ground
    public static final double goalHeight = 2.630; // meters from ground
  }

  public static final class Limelight {
    public static final double mountingAngle = 40.0; // degrees up from horizontal
    public static final double mountingHeight =
        0.7; // TODO find correct number - meters from ground

    public static final LimelightMountDirection mountingDirection =
        LimelightMountDirection.kLandscape;

    public enum LimelightMountDirection {
      kLandscape,
      kPortrait;
    }
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
