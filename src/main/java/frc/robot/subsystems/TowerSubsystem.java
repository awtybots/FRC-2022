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

  private State m_state = State.Idle;
  private boolean shouldFeed = false;

  private final WPI_TalonSRX upperMotor, lowerMotor;

  private static final double kIntakingSpeedLower = 0.4;
  private static final double kIntakingSpeedUpper = 0.3;

  private static final double kReversingSpeedLower = 0.5;
  private static final double kReversingSpeedUpper = 0.4;

  private static final double kShootingSpeedLower = 0.3;
  private static final double kShootingSpeedUpper = 0.75;

  public TowerSubsystem() {
    upperMotor = new WPI_TalonSRX(Tower.kUpperMotorCanId);
    lowerMotor = new WPI_TalonSRX(Tower.kLowerMotorCanId);

    configMotors();

    stopBoth();
  }

  private void configMotors() {
    upperMotor.configFactoryDefault();
    lowerMotor.configFactoryDefault();

    upperMotor.setInverted(false);
    lowerMotor.setInverted(false);

    upperMotor.configVoltageCompSaturation(12.0);
    lowerMotor.configVoltageCompSaturation(12.0);

    upperMotor.setNeutralMode(NeutralMode.Brake);
    lowerMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void stop() {
    m_state = State.Idle;
  }

  public void reverse() {
    m_state = State.Reversing;
  }

  public void ingest() {
    m_state = State.Loading;
  }

  public void feed(boolean ready) {
    m_state = State.Feeding;
    shouldFeed = ready;
  }

  private void intake() {
    lowerMotor.set(ControlMode.PercentOutput, kIntakingSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, kIntakingSpeedUpper);
  }

  private void reverseBoth() {
    lowerMotor.set(ControlMode.PercentOutput, -kReversingSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, -kReversingSpeedUpper);
  }

  /** only runs upper tower */
  private void feedFromUpper() {
    lowerMotor.set(ControlMode.PercentOutput, 0.0);
    upperMotor.set(ControlMode.PercentOutput, kShootingSpeedUpper);
  }

  /** runs both parts of tower */
  private void feedFromLower() {
    lowerMotor.set(ControlMode.PercentOutput, kShootingSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, kShootingSpeedUpper);
  }

  private void stopUpper() {
    upperMotor.set(ControlMode.PercentOutput, 0.0);
  }

  private void stopBoth() {
    lowerMotor.set(ControlMode.PercentOutput, 0.0);
    upperMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public boolean upperBallOurs() {
    return true;
  }

  private boolean upperBallPresent() {
    return true;
  }

  private boolean lowerBallPresent() {
    return true;
  }

  @Override
  public void periodic() {
    boolean upperPresent = upperBallPresent();
    boolean lowerPresent = lowerBallPresent();

    switch (m_state) {
      case Idle:
        stopBoth();

      case Reversing:
        reverseBoth();

      case Feeding:
        if (!shouldFeed) {
          if (lowerPresent && !upperPresent) ingest();
        } else {
          if (upperPresent) feedFromUpper();
          if (lowerPresent) feedFromLower();
        }

      case Loading:
        if (upperPresent) stopUpper();
        if (upperPresent && lowerPresent) stopBoth();
        if (!upperPresent && !lowerPresent) intake();
    }
  }

  enum State {
    Reversing,
    Idle,
    Feeding,
    Loading,
  }
}
