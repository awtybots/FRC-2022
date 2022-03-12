// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.util.math.Convert;
import frc.robot.util.math.Convert.Encoder;

public class DrivetrainSubsystem extends SubsystemBase {

  private static final double kGearRatio = 8.0 / 36.0 * 18.0 / 36.0;
  private static final double kWheelDiameter = Convert.inchesToMeters(6.0);

  private static final double kTrackWidth = Convert.inchesToMeters(22.5);
  public static final double kS = 0.0;
  public static final double kV = 0.0; // ! TODO sysId - calculateKF(velAtPercentOut, 0.5)
  public static final double kA = 0.0;
  public static final double kP = 0.0;

  public static final DifferentialDriveKinematics kKinematics =
      new DifferentialDriveKinematics(kTrackWidth);
  public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
  public static final double kTrajectoryMaxVelocity = 6.0; // m/s
  public static final double kTrajectoryMaxAcceleration = 10.0; // m/s^2
  public static final double kMaxTrajectoryVoltage = 10.0;
  private static final double kRamp = 0.25; //

  private final WPI_TalonFX leftFront, leftBack, rightFront, rightBack;
  private final WPI_TalonFX[] leftMotors, rightMotors;
  private final WPI_TalonFX[] allMotors;

  private final MotorControllerGroup leftDrive, rightDrive;
  private final DifferentialDrive drivetrain;

  private final DifferentialDriveOdometry odometry;

  private final AHRS navX = new AHRS(Port.kMXP);

  public DrivetrainSubsystem() {
    leftFront = new WPI_TalonFX(Drivetrain.kLeftFrontCanId);
    leftBack = new WPI_TalonFX(Drivetrain.kLeftBackCanId);
    rightFront = new WPI_TalonFX(Drivetrain.kRightFrontCanId);
    rightBack = new WPI_TalonFX(Drivetrain.kRightBackCanId);

    leftMotors = new WPI_TalonFX[] {leftFront, leftBack};
    rightMotors = new WPI_TalonFX[] {rightFront, rightBack};
    allMotors = new WPI_TalonFX[] {leftFront, leftBack, rightFront, rightBack};

    configMotors();

    leftDrive = new MotorControllerGroup(leftFront, leftBack);
    rightDrive = new MotorControllerGroup(rightFront, rightBack);

    drivetrain = new DifferentialDrive(leftDrive, rightDrive);

    odometry = new DifferentialDriveOdometry(new Rotation2d());
  }

  public void initOdometry(Pose2d initialPose) {
    odometry.resetPosition(initialPose, new Rotation2d(0.0));

    navX.reset();
    for (WPI_TalonFX motor : allMotors) {
      motor.setSelectedSensorPosition(0.0);
    }
  }

  private void configMotors() {
    for (WPI_TalonFX motor : allMotors) {
      motor.configFactoryDefault();

      motor.configOpenloopRamp(kRamp);
      motor.configClosedloopRamp(kRamp);
      motor.configVoltageCompSaturation(12.0);
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      motor.setSelectedSensorPosition(0.0);
      motor.setNeutralMode(NeutralMode.Brake);
    }

    rightFront.setInverted(TalonFXInvertType.Clockwise);
    leftBack.setInverted(TalonFXInvertType.Clockwise);
  }

  @Override
  public void periodic() {
    double leftDistance = getAverageDistance(leftMotors); // meters
    double rightDistance = getAverageDistance(rightMotors); // meters
    double gyroAngle = -navX.getAngle(); // degrees

    odometry.update(new Rotation2d(Math.toRadians(gyroAngle)), leftDistance, rightDistance);

    if (Constants.TUNING_MODE) {
      SmartDashboard.putNumber("DT - gyro", gyroAngle);

      if(DriverStation.isAutonomous()) {
        System.out.println("drive speed: " + getAverageSpeed(allMotors) + " m/s"); // TODO for first auton only
      }
      SmartDashboard.putNumber("DT - left vel", getAverageSpeed(leftMotors));
      SmartDashboard.putNumber("DT - right vel", getAverageSpeed(rightMotors));
    }
  }

  /** for ramsete controller */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getAverageSpeed(leftMotors), getAverageSpeed(rightMotors));
  }

  /** for ramsete controller */
  public double getHeading() {
    return navX.getYaw();
  }

  /** for ramsete controller */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** for ramsete controller */
  public void driveVolts(double leftVolts, double rightVolts) {
    for (WPI_TalonFX motor : leftMotors) motor.setVoltage(leftVolts);
    for (WPI_TalonFX motor : rightMotors) motor.setVoltage(rightVolts);
    drivetrain.feed();
  }

  /** for moving shots */
  public double getSpeed() {
    return getAverageSpeed(allMotors); // meters per second
  }

  /** meters per second */
  private double getAverageSpeed(WPI_TalonFX[] motors) {
    double sum = 0.0;
    for (WPI_TalonFX motor : motors) {
      sum +=
          Convert.encoderVelToSpeed(
              motor.getSelectedSensorVelocity(),
              kGearRatio,
              kWheelDiameter,
              Encoder.TalonFXIntegrated);
    }
    return sum / motors.length;
  }

  /** meters */
  private double getAverageDistance(WPI_TalonFX[] motors) {
    double sum = 0.0;
    for (WPI_TalonFX motor : motors) {
      sum +=
          Convert.encoderPosToDistance(
              motor.getSelectedSensorPosition(),
              kGearRatio,
              kWheelDiameter,
              Encoder.TalonFXIntegrated);
    }
    return sum / motors.length;
  }

  public void drive(double forward, double rotate) {
    // We use this because it behaves like a differential in a car.
    // The forward velocity of the robot is maintained through a turn, only
    // adjusting the turn rate.
    // in short: leftspeed = forward + rotate; rightspeed = forward - rotate
    drivetrain.curvatureDrive(forward, rotate, true);

    // If desired, during a turn, the forward speed of the robot can be reduced
    // by only adjusting one wheel speed. For example, during a left turn, the
    // right wheel speed remains at the straightline speed while the left wheel
    // slows down to make the turn. this reduces the forward speed of the robot.
    // In short, on a left turn: leftspeed = forward - rotate; rightspeed = forward
    // This behaviour can be enabled by uncommenting the line below.
    // drivetrain.arcadeDrive(forward, rotate, true);
  }

  public void stop() {
    drivetrain.tankDrive(0.0, 0.0);
  }

  private static double calculateKF(double velAtPercentOut, double percentOut) {
    // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#how-to-calculate-kf
    double sensorVelAtPercentOut =
        Convert.speedToEncoderVel(velAtPercentOut, kGearRatio, kWheelDiameter, Encoder.TalonFXIntegrated);
    return (percentOut * 1023) / sensorVelAtPercentOut;
  }
}
