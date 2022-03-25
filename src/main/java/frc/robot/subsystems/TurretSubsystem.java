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
import frc.util.math.Convert;
import frc.util.math.Convert.Encoder;

public class TurretSubsystem extends SubsystemBase {

  private static final double kAngleMin = -120.0;
  private static final double kAngleMax = 230.0;
  private static final double kAngleHalf = (kAngleMin + kAngleMax) / 2.0;
  private static final double kAngleStart = 180.0;

  public static final double kSpitAngle = 45.0;
  public static final double kSpitRelativeAngle = 45.0;

  private static final double kMaxAcceptableAngleError = 5.0;
  private static final double kManualEndzoneSize = 5.0;

  private static final double kManualPeakPercentOutput = 0.4;

  private static final double kClosedLoopPeakOutput = 0.5;
  private static final double kMaxAngularSpeed = 90.0;
  private static final double kMaxAngularAccel = 90.0;

  private static final double kGearRatio = -1.0 / 4.0 / 10.8;

  private static final double kP = 0.15;
  private static final double kI = 0.001;
  private static final double kD = 3.5;
  private static final double kIZone = 500.0;

  private final WPI_TalonSRX mMotor;

  private double mCurrentAngle = kAngleStart;
  private double mTargetAngle = mCurrentAngle;

  private State mState = State.kIdle;

  private enum State {
    kIdle,
    kSeekingRight,
    kSeekingLeft,
    kTracking,
    kSpittingAbsolute,
    kSpittingRelative,
    kPresetAngle,
    kManualOverride;
  }

  public TurretSubsystem() {
    mMotor = new WPI_TalonSRX(Turret.kMotorCanId);
    configMotors();
  }

  public void initPosition(double angle) {
    mMotor.setSelectedSensorPosition(
        Convert.angleToEncoderPos(angle, kGearRatio, Encoder.VersaPlanetaryIntegrated));
    mCurrentAngle = angle;
    mTargetAngle = angle;
  }

  private void configMotors() {
    mMotor.configFactoryDefault();

    mMotor.configVoltageCompSaturation(12.0);
    mMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    mMotor.setNeutralMode(NeutralMode.Brake);
    mMotor.setSensorPhase(true);

    mMotor.config_kP(0, kP);
    mMotor.config_kI(0, kI);
    mMotor.config_kD(0, kD);
    mMotor.config_IntegralZone(0, kIZone);
    mMotor.configClosedLoopPeakOutput(0, kClosedLoopPeakOutput);
    mMotor.configMotionCruiseVelocity(
        Convert.angularVelToEncoderVel(
            kMaxAngularSpeed, kGearRatio, Encoder.VersaPlanetaryIntegrated));
    mMotor.configMotionAcceleration(
        Convert.angularAccelToEncoderAccel(
            kMaxAngularAccel, kGearRatio, Encoder.VersaPlanetaryIntegrated));

    // SmartDashboard.putNumber("TU - kP", kP);
    // SmartDashboard.putNumber("TU - kI", kI);
    // SmartDashboard.putNumber("TU - kIZone", kIZone);
    // SmartDashboard.putNumber("TU - kD", kD);
    // SmartDashboard.putNumber("TU - kMaxAngularSpeed", kMaxAngularSpeed);
    // SmartDashboard.putNumber("TU - kMaxAngularAccel", kMaxAngularAccel);

    mMotor.setSelectedSensorPosition(
        Convert.angleToEncoderPos(kAngleStart, kGearRatio, Encoder.VersaPlanetaryIntegrated));
  }

  @Override
  public void periodic() {
    mCurrentAngle = getAngle();
    SmartDashboard.putNumber("TU - actual angle", mCurrentAngle);

    if (Constants.TUNING_MODE) {
      SmartDashboard.putBoolean("TU - at goal", isAtTarget());
      SmartDashboard.putNumber("TU - target angle", mTargetAngle);
      SmartDashboard.putString("TU - state", mState.toString());

      // mMotor.config_kP(0, SmartDashboard.getNumber("TU - kP", kP));
      // mMotor.config_kI(0, SmartDashboard.getNumber("TU - kI", kI));
      // mMotor.config_IntegralZone(0, SmartDashboard.getNumber("TU - kIZone", kIZone));
      // mMotor.config_kD(0, SmartDashboard.getNumber("TU - kD", kD));
      // mMotor.configMotionCruiseVelocity(
      //     Convert.angularVelToEncoderVel(
      //         SmartDashboard.getNumber("TU - kMaxAngularSpeed", kMaxAngularSpeed),
      //         kGearRatio,
      //         Encoder.VersaPlanetaryIntegrated));
      // mMotor.configMotionAcceleration(
      //     Convert.angularAccelToEncoderAccel(
      //         SmartDashboard.getNumber("TU - kMaxAngularAccel", kMaxAngularAccel),
      //         kGearRatio,
      //         Encoder.VersaPlanetaryIntegrated));
      // SmartDashboard.putNumber(
      //     "TU - angular velocity",
      //     Convert.encoderVelToAngularVel(
      //         mMotor.getSelectedSensorVelocity(), kGearRatio, Encoder.VersaPlanetaryIntegrated));
    }
  }

  // PRIVATE HELPER METHODS

  private void turnToPrivate(double absoluteAngle) {
    mTargetAngle = clampToBounds(absoluteAngle);

    mMotor.set(
        ControlMode.MotionMagic,
        Convert.angleToEncoderPos(mTargetAngle, kGearRatio, Encoder.VersaPlanetaryIntegrated));
  }

  /** this is only for use in the subsystem once per frame to prevent CAN overloading */
  private double getAngle() {
    return Convert.encoderPosToAngle(
        mMotor.getSelectedSensorPosition(), kGearRatio, Encoder.VersaPlanetaryIntegrated);
  }

  private double clampToBounds(double angle) {
    return MathUtil.clamp(angle, kAngleMin, kAngleMax);
  }

  // PUBLIC METHODS

  public void turnTo(double absoluteAngle) {
    mState = State.kPresetAngle;
    turnToPrivate(absoluteAngle);
  }

  public void trackTarget(boolean hasTarget) {
    trackTarget(hasTarget, 0.0);
  }

  public void trackTarget(boolean hasTarget, double targetOffsetAngle) {
    double boundedGoalAngle = clampToBounds(mCurrentAngle + targetOffsetAngle);

    if (mState == State.kIdle) {
      mState = State.kTracking;
    }

    if (mState == State.kTracking) {
      if (hasTarget) {
        turnToPrivate(boundedGoalAngle);
      } else {
        if (mCurrentAngle > kAngleHalf) {
          mState = State.kSeekingRight;
          turnToPrivate(kAngleMax);
        } else {
          mState = State.kSeekingLeft;
          turnToPrivate(kAngleMin);
        }
      }
    } else if (mState == State.kSeekingLeft || mState == State.kSeekingRight) {
      if (hasTarget) {
        mState = State.kTracking;
        turnToPrivate(boundedGoalAngle);
      } else if (isAtTarget()) {
        if (mState == State.kSeekingLeft) {
          mState = State.kSeekingRight;
          turnToPrivate(kAngleMax);
        } else if (mState == State.kSeekingRight) {
          mState = State.kSeekingLeft;
          turnToPrivate(kAngleMin);
        }
      }
    }
  }

  public void spit(boolean hasTarget, double targetOffsetAngle) {
    if (mState == State.kSpittingRelative || mState == State.kSpittingAbsolute) return;

    if (hasTarget) {
      mState = State.kSpittingRelative;

      double boundedAngle = clampToBounds(mCurrentAngle + targetOffsetAngle + kSpitRelativeAngle);
      if (boundedAngle == kAngleMax) {
        boundedAngle = clampToBounds(mCurrentAngle + targetOffsetAngle - kSpitRelativeAngle);
      }

      turnToPrivate(boundedAngle);
    } else {
      mState = State.kSpittingAbsolute;

      if (mCurrentAngle > 0.0) {
        turnToPrivate(kSpitAngle);
      } else {
        turnToPrivate(-kSpitAngle);
      }
    }
  }

  public void idle() {
    mState = State.kIdle;

    mMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void drive(double rate) {
    mState = State.kManualOverride;

    rate = MathUtil.clamp(rate, -1.0, 1.0);
    boolean atMinBound = mCurrentAngle <= kAngleMin + kManualEndzoneSize;
    boolean atMaxBound = mCurrentAngle >= kAngleMax - kManualEndzoneSize;

    double pctOut = rate * kManualPeakPercentOutput;

    if ((atMinBound && rate > 0) || (atMaxBound && rate < 0)) {
      pctOut = 0.0;
    }
    mMotor.set(ControlMode.PercentOutput, pctOut);
  }

  public boolean isAtTarget() {
    return Math.abs(mCurrentAngle - mTargetAngle) < kMaxAcceptableAngleError;
  }

  /** public accessor method */
  public double getCurrentAngle() {
    return mCurrentAngle;
  }
}
