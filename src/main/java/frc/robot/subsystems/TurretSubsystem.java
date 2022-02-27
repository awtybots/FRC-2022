// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Turret;

public class TurretSubsystem extends SubsystemBase {

  private final WPI_TalonSRX motor;
  private final double kMaxDegPerSec = 10;

  public TurretSubsystem() {
    motor = new WPI_TalonSRX(Turret.kMotor);
    configMotors();
  }

  private void configMotors() {
    motor.configAllSettings(Turret.motorConfig());
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    motor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // ! stop if turret not in bounds
    // if (motor.getSelectedSensorPosition())
  }

  /** positive is clockwise (-1 to 1) */
  public void drive(double rate) {
    motor.set(ControlMode.PercentOutput, rate * kMaxDegPerSec);
  }
}
