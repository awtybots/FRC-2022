// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
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

  private final double kGearRatio = 8.0 / 36.0 * 18.0 / 36.0;
  private final double kWheelDiameter = Convert.inchesToMeters(6.0);

  private final WPI_TalonFX leftFront, leftBack, rightFront, rightBack;
  private final WPI_TalonFX[] leftMotors, rightMotors;
  private final WPI_TalonFX[] allMotors;

  private final MotorControllerGroup leftDrive, rightDrive;
  private final DifferentialDrive drivetrain;

  private final DifferentialDriveOdometry odometry;

  private final AHRS navX = new AHRS(Port.kMXP);

  public DrivetrainSubsystem() {
    leftFront = new WPI_TalonFX(Drivetrain.kLeftFront);
    leftBack = new WPI_TalonFX(Drivetrain.kLeftBack);
    rightFront = new WPI_TalonFX(Drivetrain.kRightFront);
    rightBack = new WPI_TalonFX(Drivetrain.kRightBack);

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
    odometry.resetPosition(initialPose, new Rotation2d(Math.toRadians(navX.getAngle())));

    for (WPI_TalonFX motor : allMotors) {
      motor.setSelectedSensorPosition(0.0);
    }
  }

  private void configMotors() {
    for (WPI_TalonFX motor : allMotors) {
      motor.configFactoryDefault();

      motor.configOpenloopRamp(0.1);
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
    double gyroAngle = navX.getAngle(); // ! TODO

    odometry.update(new Rotation2d(Math.toRadians(gyroAngle)), leftDistance, rightDistance);

    if (Constants.TUNING_MODE) {
      SmartDashboard.putNumber("DT - left distance", leftDistance);
      SmartDashboard.putNumber("DT - right distance", rightDistance);
      SmartDashboard.putNumber("DT - gyro", gyroAngle);

      SmartDashboard.putNumber("DT - x", navX.getDisplacementX());
      SmartDashboard.putNumber("DT - y", navX.getDisplacementY());
      SmartDashboard.putNumber("DT - z", navX.getDisplacementZ());
    }
  }

  public double getSpeed() {
    return getAverageSpeed(allMotors); // meters per second
  }

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
}
