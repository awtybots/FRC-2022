// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Tower;

public class TowerSubsystem extends SubsystemBase {

  private final WPI_TalonSRX upperMotor, lowerMotor;

  private final double kLowerMotorSpeed = 0.6; // TODO
  private final double kUpperMotorSpeedIntaking = 0.3; // TODO
  private final double kUpperMotorSpeedShooting = 0.9; // TODO

  public TowerSubsystem() {
    upperMotor = new WPI_TalonSRX(Tower.kUpperMotor);
    lowerMotor = new WPI_TalonSRX(Tower.kLowerMotor);

    configMotors();

    stop();
  }

  private void configMotors() {
    upperMotor.configAllSettings(Tower.motorConfig());
    lowerMotor.configAllSettings(Tower.motorConfig());

    upperMotor.setInverted(false);
    lowerMotor.setInverted(false);

    upperMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void startForIntaking() {
    lowerMotor.set(ControlMode.PercentOutput, kLowerMotorSpeed);
    upperMotor.set(ControlMode.PercentOutput, kUpperMotorSpeedIntaking);
  }

  public void startForShooting() {
    lowerMotor.set(ControlMode.PercentOutput, kLowerMotorSpeed);
    upperMotor.set(ControlMode.PercentOutput, kUpperMotorSpeedShooting);
  }

  public void stopUpper() {
    upperMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void stop() {
    lowerMotor.set(ControlMode.PercentOutput, 0.0);
    upperMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
