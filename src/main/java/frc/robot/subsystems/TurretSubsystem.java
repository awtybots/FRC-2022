// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Turret;
import frc.robot.util.math.Convert;
import frc.robot.util.math.Convert.Encoder;

public class TurretSubsystem extends SubsystemBase {

  private final double kAngleMin = -165.0;
  private final double kAngleMax = 195.0;
  private final double kAngleStart = 0.0; // TODO best start angle

  private final double kGearRatio = 1.0; // TODO

  private final double kP = 0.0;
  private final double kD = 0.0;
  private final double kMaxDegPerSec = 10.0;
  private final double kMaxDegPerSecPerSec = 0.0;

  private final double kMaxManualPercentOutput = 10.0;

  private final WPI_TalonSRX motor;

  public TurretSubsystem() {
    motor = new WPI_TalonSRX(Turret.kMotor);
    configMotors();
  }

  private void configMotors() {
    motor.configVoltageCompSaturation(12.0);
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    motor.config_kP(0, kP);
    motor.config_kD(0, kD);
    motor.configMotionCruiseVelocity(
        Convert.degPerSecToEncoderVel(kMaxDegPerSec, Encoder.VersaPlanetaryIntegrated));
    motor.configMotionAcceleration(
        Convert.degPerSecPerSecToEncoderAccel(
            kMaxDegPerSecPerSec, Encoder.VersaPlanetaryIntegrated));

    motor.setSelectedSensorPosition(
        0); // TODO use gear ratio to set start at actual start in case it's not 0
  }

  @Override
  public void periodic() {
    // ! stop if turret not in bounds
    // if (motor.getSelectedSensorPosition())
  }

  /** positive is clockwise (-1 to 1) */
  public void drive(double rate) {
    motor.set(ControlMode.PercentOutput, rate * kMaxManualPercentOutput);
  }

  // TODO automatic turret angle

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }
}
