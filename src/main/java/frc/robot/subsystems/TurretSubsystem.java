// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Turret;

public class TurretSubsystem extends SubsystemBase {

  private final WPI_TalonSRX motor;

  public TurretSubsystem() {
    motor = new WPI_TalonSRX(Turret.kMotor);
    configMotors();
  }

  private void configMotors() {
    motor.configAllSettings(Turret.motorConfig());
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
  }
}
