// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Turret;
import frc.robot.util.math.Convert;
import frc.robot.util.math.Convert.Encoder;

public class TurretSubsystem extends SubsystemBase {

  private final double kAngleMin = -135.0;
  private final double kAngleMax = 225.0;
  private final double kAngleStart = 0.0;

  private final double kGearRatio = 1.0 / 10.8;

  private final double kP = 0.0;
  private final double kF = 0.0;
  private final double kMaxDegPerSec = 10.0;
  private final double kMaxDegPerSecPerSec = 0.0;

  private final double kMaxAcceptableAngleError = 1.0;

  private final double kMaxManualPercentOutput = 0.2;

  private final WPI_TalonSRX motor;

  private double actualAngle = kAngleStart;
  private double targetAngle = actualAngle;

  public TurretSubsystem() {
    motor = new WPI_TalonSRX(Turret.kMotorCanId);
    configMotors();

    if (Constants.TUNING_MODE) {
      SmartDashboard.putNumber("TU - set target angle", kAngleStart);
      SmartDashboard.putNumber("TU - P", kP);
      SmartDashboard.putNumber("TU - F", kF);
    }
  }

  public void initPosition(double angle) {
    motor.setSelectedSensorPosition(
        Convert.angleToEncoderPos(angle, kGearRatio, Encoder.VersaPlanetaryIntegrated));
    actualAngle = angle;
    targetAngle = angle;
  }

  private void configMotors() {
    motor.configFactoryDefault();

    motor.configVoltageCompSaturation(12.0);
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    motor.setNeutralMode(NeutralMode.Brake);

    motor.config_kP(0, kP);
    motor.config_kF(0, kF);
    motor.configMotionCruiseVelocity(
        Convert.angularVelToEncoderVel(
            kMaxDegPerSec, kGearRatio, Encoder.VersaPlanetaryIntegrated));
    motor.configMotionAcceleration(
        Convert.angularAccelToEncoderAccel(
            kMaxDegPerSecPerSec, kGearRatio, Encoder.VersaPlanetaryIntegrated));

    motor.setSelectedSensorPosition(
        Convert.angleToEncoderPos(kAngleStart, kGearRatio, Encoder.VersaPlanetaryIntegrated));
  }

  @Override
  public void periodic() {
    actualAngle = getAngle();

    if (actualAngle < kAngleMin - kMaxAcceptableAngleError
        || actualAngle > kAngleMax + kMaxAcceptableAngleError) {
      stop(); // ? maybe remove for competition
    }

    if (Constants.TUNING_MODE) {
      turnTo(SmartDashboard.getNumber("TU - set target angle", targetAngle)); // ! remove

      motor.config_kP(0, SmartDashboard.getNumber("TU - P", kP));
      motor.config_kF(0, SmartDashboard.getNumber("TU - F", kF));

      SmartDashboard.putBoolean("TU - at goal", isAtTarget());
      SmartDashboard.putNumber("TU - actual angle", actualAngle);
      SmartDashboard.putNumber("TU - target angle", targetAngle);
    }
  }

  public void turnBy(double deltaAngle) {
    turnTo(actualAngle + deltaAngle);
  }

  public void turnTo(double absoluteAngle) {
    while (absoluteAngle < kAngleMin) {
      absoluteAngle += 360.0;
    }
    while (absoluteAngle > kAngleMax) {
      absoluteAngle -= 360.0;
    }
    targetAngle = MathUtil.clamp(absoluteAngle, kAngleMin, kAngleMax);
    motor.set(
        ControlMode.Position,
        Convert.angleToEncoderPos(targetAngle, kGearRatio, Encoder.VersaPlanetaryIntegrated));
  }

  public boolean isAtTarget() {
    return Math.abs(actualAngle - targetAngle) < kMaxAcceptableAngleError;
  }

  /** this is only for use in the subsystem once per frame to prevent CAN overloading */
  private double getAngle() {
    return Convert.encoderPosToAngle(
        motor.getSelectedSensorPosition(), kGearRatio, Encoder.VersaPlanetaryIntegrated);
  }

  /** public accessor method */
  public double getActualAngle() {
    return actualAngle;
  }

  /** positive is clockwise (-1 to 1) */
  public void drive(double rate) {
    motor.set(ControlMode.PercentOutput, rate * kMaxManualPercentOutput);
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }
}
