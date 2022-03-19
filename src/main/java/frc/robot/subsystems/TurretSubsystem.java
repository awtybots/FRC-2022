// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Turret;
import frc.robot.util.math.Convert;
import frc.robot.util.math.Convert.Encoder;

public class TurretSubsystem extends SubsystemBase {

  private Mode tMode = Mode.Idle;

  private static final double kAngleMin = -135.0;
  private static final double kAngleMax = 225.0;
  private static final double kAngleStart = 180.0;

  public static final double kSpitAngle = 90.0;
  public static final double kSpitRelativeAngle = 40.0;

  private static final double kGearRatio = -1.0 / 4.0 / 10.8;

  private static final double kP = 0.2;
  private static final double kI = 0.002;

  private static final double kMaxAcceptableAngleError = 3.0;

  private static final double kPeakOutput = 0.5;
  private static final double kMaxDrivePercentOutput = kPeakOutput;

  private final WPI_TalonSRX motor;

  private double actualAngle = kAngleStart;
  private double targetAngle = actualAngle;

  private boolean seeking = false;
  private boolean seekingRight = true;

  public TurretSubsystem() {
    motor = new WPI_TalonSRX(Turret.kMotorCanId);
    configMotors();
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
    motor.setSensorPhase(true);

    motor.config_kP(0, kP);
    motor.config_kI(0, kI);
    motor.config_IntegralZone(0, 1000);
    motor.configClosedLoopPeakOutput(0, kPeakOutput);
    motor.configMotionCruiseVelocity(
        Convert.angularVelToEncoderVel(180.0, kGearRatio, Encoder.VersaPlanetaryIntegrated));
    motor.configMotionAcceleration(
        Convert.angularAccelToEncoderAccel(90.0, kGearRatio, Encoder.VersaPlanetaryIntegrated));

    motor.setSelectedSensorPosition(
        Convert.angleToEncoderPos(kAngleStart, kGearRatio, Encoder.VersaPlanetaryIntegrated));
  }

  @Override
  public void periodic() {
    actualAngle = getAngle();
    SmartDashboard.putNumber("TU - actual angle", actualAngle);
    SmartDashboard.putData("Turret", this);

    if (Constants.TUNING_MODE) {
      // turnTo(SmartDashboard.getNumber("TU - set target angle", targetAngle));
      SmartDashboard.putBoolean("TU - at goal", isAtTarget());
      SmartDashboard.putNumber("TU - target angle", targetAngle);
    }
  }

  /** spins through full range of motion continuously unless turnTo or turnBy is called */
  public void seek() {
    tMode = Mode.Targeting;
    if (seeking) {
      if (isAtTarget()) {
        seekingRight = !seekingRight;
        turnTo(seekingRight ? kAngleMax : kAngleMin);
        seeking = true;
      }
    } else {
      seekingRight = (kAngleMax - actualAngle) < 180.0;
      turnTo(seekingRight ? kAngleMax : kAngleMin);
      seeking = true;
    }
  }

  public void spit() {
    spitRelative(actualAngle);
    // turnTo(kSpitAngle);
  }

  public void spitRelative(double original) {
    turnBy(original + kSpitRelativeAngle);
  }

  public void turnBy(double deltaAngle) {
    turnTo(actualAngle + deltaAngle);
  }

  public void turnTo(double absoluteAngle) {
    tMode = Mode.ManualAngle;
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
    seeking = false;
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
    if (tMode == Mode.Recovering && !isAtTarget()) {
      turnTo(0);
    } else if (getAngle() - 5 < kAngleMin || getAngle() + 5 > kAngleMax) {
      turnTo(0);
      tMode = Mode.Recovering;
    } else {
      tMode = Mode.Idle;
      motor.set(ControlMode.PercentOutput, rate * kMaxDrivePercentOutput);
    }
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }

  public enum Mode {
    Targeting,
    ManualAngle,
    Idle,
    Recovering;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty(
        "Mode",
        () -> {
          return tMode.toString();
        },
        (String a) -> {});
  }
}
