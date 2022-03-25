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

  private static final double kIntakingSpeedLower = 0.45;
  private static final double kIntakingSpeedUpper = 0.3;

  private static final double kReversingSpeedLower = 0.45;
  private static final double kReversingSpeedUpper = 0.3;

  private static final double kShootingSpeedLower = 0.3;
  private static final double kShootingSpeedUpper = 0.75;

  public TowerSubsystem() {
    upperMotor = new WPI_TalonSRX(Tower.kUpperMotorCanId);
    lowerMotor = new WPI_TalonSRX(Tower.kLowerMotorCanId);

    configMotors();

    stop();
  }

  private void configMotors() {
    upperMotor.configFactoryDefault();
    lowerMotor.configFactoryDefault();

    upperMotor.setInverted(false);
    lowerMotor.setInverted(false);

    upperMotor.setNeutralMode(NeutralMode.Brake);
    lowerMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void intake() {
    lowerMotor.set(ControlMode.PercentOutput, kIntakingSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, kIntakingSpeedUpper);
  }

  public void reverseBoth() {
    lowerMotor.set(ControlMode.PercentOutput, -kReversingSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, -kReversingSpeedUpper);
  }

  /** only runs upper tower */
  public void feedShooter1() {
    lowerMotor.set(ControlMode.PercentOutput, 0.0);
    upperMotor.set(ControlMode.PercentOutput, kShootingSpeedUpper);
  }

  /** runs both parts of tower */
  public void feedShooter2() {
    lowerMotor.set(ControlMode.PercentOutput, kShootingSpeedLower);
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
