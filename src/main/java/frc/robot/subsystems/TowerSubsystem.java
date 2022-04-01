// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Tower;

public class TowerSubsystem extends SubsystemBase {

  private final DoubleSolenoid pistons;
  private final WPI_TalonSRX upperMotor;
  private final WPI_TalonFX lowerMotor;

  private static final double kIntakingSpeedLower = 0.4;
  private static final double kIntakingSpeedUpper = 0.3;

  private static final double kReversingSpeedLower = 0.5;
  private static final double kReversingSpeedUpper = 0.4;

  private static final double kShootingSpeedLower = 0.3;
  private static final double kShootingSpeedUpper = 0.75;

  public TowerSubsystem() {
    pistons =
        new DoubleSolenoid(PneumaticsModuleType.REVPH, Tower.kSolenoidDown, Tower.kSolenoidUp);
    upperMotor = new WPI_TalonSRX(Tower.kUpperMotorCanId);
    lowerMotor = new WPI_TalonFX(Tower.kLowerMotorCanId);

    configMotors();

    stop();
  }

  private void configMotors() {
    upperMotor.configFactoryDefault();
    lowerMotor.configFactoryDefault();

    upperMotor.setInverted(false);
    lowerMotor.setInverted(true);

    upperMotor.configVoltageCompSaturation(12.0);
    lowerMotor.configVoltageCompSaturation(12.0);

    upperMotor.setNeutralMode(NeutralMode.Brake);
    lowerMotor.setNeutralMode(NeutralMode.Brake);
  }

  private void togglePistons(boolean out) {
    pistons.set(out ? Value.kForward : Value.kReverse);
  }

  public void intake() {
    lowerMotor.set(ControlMode.PercentOutput, kIntakingSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, kIntakingSpeedUpper);
    togglePistons(true);
  }

  public void reverseBoth() {
    lowerMotor.set(ControlMode.PercentOutput, -kReversingSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, -kReversingSpeedUpper);
    togglePistons(true);
  }

  /** only runs upper tower */
  public void feedFromUpper() {
    lowerMotor.set(ControlMode.PercentOutput, 0.0);
    upperMotor.set(ControlMode.PercentOutput, kShootingSpeedUpper);
  }

  /** runs both parts of tower */
  public void feedFromLower() {
    lowerMotor.set(ControlMode.PercentOutput, kShootingSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, kShootingSpeedUpper);
  }

  public void stopUpper() {
    upperMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void stop() {
    lowerMotor.set(ControlMode.PercentOutput, 0.0);
    upperMotor.set(ControlMode.PercentOutput, 0.0);
    togglePistons(false);
  }
}
