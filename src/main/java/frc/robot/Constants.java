// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.math.Vector2;
import java.util.HashMap;

public final class Constants {
  public static final boolean TUNING_MODE = true; // * TODO disable debug mode for competition

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
    public static final int kFlywheelMotor = 10;

    public static TalonFXConfiguration motorConfig() {
      TalonFXConfiguration conf = new TalonFXConfiguration();
      conf.voltageCompSaturation = 12.0;
      return conf;
    }
  }

  public static final class Field {
    public static final double kVisionTargetHeight = 2.605; // meters from ground
    public static final double kGoalHeight = 2.630; // meters from ground

    public static final double kBallRadius = 0.120; // meters
    public static final double kBallMass = 0.270; // kg
    public static final HashMap<Alliance, Color> kBallColors = new HashMap<>();

    static { // ! FIXME set correct color for blue and red ball
      kBallColors.put(Alliance.Blue, new Color(0.0, 0.0, 1.0));
      kBallColors.put(Alliance.Red, new Color(1.0, 0.0, 0.0));
    }
  }

  public static final class Limelight {
    public static final double kMountingAngle = 40.0; // degrees up from horizontal
    public static final double kMountingHeight = 0.991; // meters from ground
    public static final Vector2 kShooterOffset = new Vector2(0.152, -0.152);

    public static final LimelightMountDirection kMountingDirection =
        LimelightMountDirection.kLandscape;

    // ! TODO create limelight pipelines on the limelight dashboard
    public static final int kPipelineDriving = 0;
    public static final int kPipelineShooting = 1;

    public enum LimelightMountDirection {
      kLandscape,
      kPortrait;
    }
  }

  public static final
  class ColorSensors { // !!! TODO color sensors dont want to turn on, please fix
    public static final I2C.Port kUpperSensorPort = I2C.Port.kOnboard;
    public static final I2C.Port kLowerSensorPort = I2C.Port.kMXP;
  }

  public static final class Intake {
    public static final int kMotor = 8;
    public static final int kSolenoidUp = 1; // ! FIXME change to correct Pneumatics Hub port
    public static final int kSolenoidDown = 2; // ! FIXME change to correct Pneumatics Hub port

    public static TalonSRXConfiguration motorConfig() {
      TalonSRXConfiguration conf = new TalonSRXConfiguration();
      return conf;
    }
  }

  public static final class Climber {
    public static final int kLeftMotor = 18; // TODO change to correct motor ID
    public static final int kRightMotor = 18; // TODO change to correct motor ID

    public static TalonFXConfiguration motorConfig() {
      TalonFXConfiguration conf = new TalonFXConfiguration();
      return conf;
    }
  }
}
