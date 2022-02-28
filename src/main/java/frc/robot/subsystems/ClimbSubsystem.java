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

  private final double kP = 0.0;
  private final double kD = 0.0;
  private final double kMaxSpeed = Convert.inchesToMeters(15.0); // TODO tune
  private final double kMaxAccel = Convert.inchesToMeters(15.0); // TODO
  private final double kMaxPercentOutput = 0.5;

  private final double kClimbDistance = Convert.inchesToMeters(25.0); // TODO correct values
  private final double kWinchDiameter = Convert.inchesToMeters(1.5); // TODO
  private final double kGearRatio = 1.0; // TODO

  private final double kMaxAcceptablePositionError = Convert.inchesToMeters(0.5);

  private double targetPosition = 0.0;
  private double actualPosition = 0.0;

  public ClimbSubsystem() {
    leftMotor = new WPI_TalonFX(Climber.kLeftMotor);
    rightMotor = new WPI_TalonFX(Climber.kRightMotor);
    motors = new WPI_TalonFX[] {leftMotor, rightMotor};

    configMotors();
  }

  @Override
  public void periodic() {
    if (Constants.TUNING_MODE) {
      SmartDashboard.putBoolean("CL - at goal", isAtTarget());
      SmartDashboard.putNumber("CL - actual pos", Convert.metersToInches(actualPosition));
      SmartDashboard.putNumber("CL - target pos", Convert.metersToInches(targetPosition));
    }
  }

  public boolean isAtTarget() {
    return Math.abs(actualPosition - targetPosition) < kMaxAcceptablePositionError;
  }

  private void configMotors() {
    rightMotor.setInverted(TalonFXInvertType.Clockwise);

    for (WPI_TalonFX motor : motors) {
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      motor.setSelectedSensorPosition(0.0);

      motor.setNeutralMode(NeutralMode.Brake);

      motor.config_kP(0, kP);
      motor.config_kD(0, kD);
      motor.configMotionCruiseVelocity(
          Convert.speedToEncoderVel(
              kMaxSpeed, kGearRatio, kWinchDiameter, Encoder.TalonFXIntegrated));
      motor.configMotionAcceleration(
          Convert.accelToEncoderAccel(
              kMaxAccel, kGearRatio, kWinchDiameter, Encoder.TalonFXIntegrated));
    }
  }

  public void raiseClimb() {
    targetPosition = kClimbDistance;
    for (WPI_TalonFX motor : motors)
      motor.set(
          ControlMode.Position,
          Convert.distanceToEncoderPos(
              kClimbDistance, kGearRatio, kWinchDiameter, Encoder.TalonFXIntegrated));
  }

  public void retractClimb() {
    targetPosition = 0.0;
    for (WPI_TalonFX motor : motors) motor.set(ControlMode.Position, 0.0);
  }

  public void drive(double pct) {
    for (WPI_TalonFX motor : motors) motor.set(ControlMode.PercentOutput, pct * kMaxPercentOutput);
  }

  public void stop() {
    for (WPI_TalonFX motor : motors) motor.set(ControlMode.PercentOutput, 0.0);
  }
}
