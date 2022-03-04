// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;
import frc.robot.util.math.Convert;
import frc.robot.util.math.Convert.Encoder;

public class ClimbSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftMotor;
  private final WPI_TalonFX rightMotor;

  private final WPI_TalonFX[] motors;

  private static final double kClimbDistance = Convert.inchesToMeters(25.0);
  private static final double kWinchDiameter = Convert.inchesToMeters(0.92); // TODO tune
  private static final double kGearRatio = 1.0 / 10.0 / 10.0 * 34.0 / 44.0;

  private static final double kP = 0.0; // TODO tune
  private static final double kF = calculateKF(Convert.inchesToMeters(5.0), 0.5); // TODO correct
  private static final double kMaxSpeed = Convert.inchesToMeters(15.0);
  private static final double kMaxAccel = Convert.inchesToMeters(15.0);
  private static final double kMaxPercentOutput = 0.5;

  private static final double kMaxAcceptablePositionError = Convert.inchesToMeters(0.5);

  private double targetPosition = 0.0;
  private double actualPosition = 0.0;

  public ClimbSubsystem() {
    leftMotor = new WPI_TalonFX(Climber.kLeftMotorCanId);
    rightMotor = new WPI_TalonFX(Climber.kRightMotorCanId);
    motors = new WPI_TalonFX[] {leftMotor, rightMotor};

    configMotors();

    if (Constants.TUNING_MODE) {
      SmartDashboard.putNumber("CL - set target pos", actualPosition);
    }
  }

  @Override
  public void periodic() {
    actualPosition = getPosition();

    if (Constants.TUNING_MODE) {
      // moveClimb(
      //     Convert.inchesToMeters(
      //         SmartDashboard.getNumber(
      //             "CL - set target pos", Convert.metersToInches(targetPosition)))); // ! remove
      // after tuning

      SmartDashboard.putBoolean("CL - at goal", isAtTarget());
      SmartDashboard.putNumber("CL - actual pos", Convert.metersToInches(actualPosition));
      SmartDashboard.putNumber("CL - target pos", Convert.metersToInches(targetPosition));
    }
  }

  private double getPosition() {
    double sum = 0.0;
    for (WPI_TalonFX motor : motors) {
      sum +=
          Convert.encoderPosToDistance(
              motor.getSelectedSensorPosition(),
              kGearRatio,
              kWinchDiameter,
              Encoder.TalonFXIntegrated);
    }
    return sum / motors.length;
  }

  public boolean isAtTarget() {
    return Math.abs(actualPosition - targetPosition) < kMaxAcceptablePositionError;
  }

  private void configMotors() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    rightMotor.setInverted(TalonFXInvertType.Clockwise);

    for (WPI_TalonFX motor : motors) {
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      motor.setSelectedSensorPosition(0.0);

      motor.setNeutralMode(NeutralMode.Brake);

      motor.configOpenloopRamp(1.0);
      motor.configClosedloopRamp(1.0);
      motor.configPeakOutputForward(kMaxPercentOutput);
      motor.configPeakOutputReverse(-kMaxPercentOutput);

      motor.config_kP(0, kP);
      motor.config_kF(0, kF);
      motor.configMotionCruiseVelocity(
          Convert.speedToEncoderVel(
              kMaxSpeed, kGearRatio, kWinchDiameter, Encoder.TalonFXIntegrated));
      motor.configMotionAcceleration(
          Convert.accelToEncoderAccel(
              kMaxAccel, kGearRatio, kWinchDiameter, Encoder.TalonFXIntegrated));
    }
  }

  private void moveClimb(double pos) {
    targetPosition = pos;
    for (WPI_TalonFX motor : motors)
      motor.set(
          ControlMode.Position,
          Convert.distanceToEncoderPos(
              targetPosition, kGearRatio, kWinchDiameter, Encoder.TalonFXIntegrated));
  }

  public void raiseClimb() {
    moveClimb(kClimbDistance);
  }

  public void retractClimb() {
    moveClimb(0.0);
  }

  public void drive(double pct) {
    for (WPI_TalonFX motor : motors) motor.set(ControlMode.PercentOutput, pct * kMaxPercentOutput);
  }

  public void stop() {
    for (WPI_TalonFX motor : motors) motor.set(ControlMode.PercentOutput, 0.0);
  }

  private static double calculateKF(double speed, double percentOut) {
    // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#how-to-calculate-kf
    double sensorVelAtPercentOut = Convert.speedToEncoderVel(speed, kGearRatio, kWinchDiameter, Encoder.TalonFXIntegrated);
    return (percentOut * 1023) / sensorVelAtPercentOut;
  }
}
