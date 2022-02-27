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

  private final double kSpeedLower = 0.75; // * FIXME determine correct lower tower speed
  private final double kIntakingSpeedUpper = 0.75; // * FIXME determine correct upper tower speed
  private final double kShootingSpeedUpper = 0.9; // * FIXME determine correct upper tower speed

  public TowerSubsystem() {
    upperMotor = new WPI_TalonSRX(Tower.kUpperMotor);
    lowerMotor = new WPI_TalonSRX(Tower.kLowerMotor);

    configMotors();

    stop();
  }

  private void configMotors() {
    upperMotor.configAllSettings(Tower.motorConfig());
    lowerMotor.configAllSettings(Tower.motorConfig());

    upperMotor.setInverted(true);
    lowerMotor.setInverted(false);

    upperMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void intake() {
    lowerMotor.set(ControlMode.PercentOutput, kSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, kIntakingSpeedUpper);
  }

  public void reverseBoth() {
    lowerMotor.set(ControlMode.PercentOutput, -kSpeedLower);
    lowerMotor.set(ControlMode.PercentOutput, -kIntakingSpeedUpper);
  }

  public void feedShooter() {
    lowerMotor.set(ControlMode.PercentOutput, kSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, kShootingSpeedUpper);
  }

  public void stopUpper() {
    upperMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void stop() {
    lowerMotor.set(ControlMode.PercentOutput, 0.0);
    upperMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
