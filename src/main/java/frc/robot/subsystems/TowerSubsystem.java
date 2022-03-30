// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ColorSensors;
import frc.robot.Constants.Tower;
import frc.util.ColorSensor;

public class TowerSubsystem extends SubsystemBase {

  private State m_state = State.Idle;
  private boolean firing = false;

  private Alliance ourAlliance = Alliance.Invalid;
  private final Color kRed = new Color(0.41, 0.41, 0.18);
  private final Color kBlue = new Color(0.17, 0.41, 0.43);
  private final ColorSensor lowerSensor, upperSensor;
  private final WPI_TalonSRX upperMotor, lowerMotor;

  private static final double kLoadingSpeedLower = 0.4;
  private static final double kLoadingSpeedUpper = 0.3;

  private static final double kReversingSpeedLower = 0.5;
  private static final double kReversingSpeedUpper = 0.4;

  private static final double kShootingSpeedLower = 0.3;
  private static final double kShootingSpeedUpper = 0.75;

  public TowerSubsystem() {
    upperSensor = new ColorSensor(ColorSensors.kUpperSensorPort, kRed, kBlue, 250, 0.90);
    lowerSensor = new ColorSensor(ColorSensors.kLowerSensorPort, kRed, kBlue, 250, 0.90);

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

  private enum State {
    Reversing,
    Idle,
    Feeding,
    Loading,
  }

  @Override
  public void periodic() {
    boolean upperPresent = upperBallPresent();
    boolean lowerPresent = lowerBallPresent();

    switch (m_state) {
      case Feeding:
        if (!firing) load();
        else {
          if (!upperPresent) feedBoth();
          if (upperPresent) feedUpper();
        }

      case Loading:
        // TODO ensure there's no balls in the tower before we send lower one up
        if (!upperPresent) loadBoth();
        if (upperPresent && !lowerPresent) loadLower();
        if (upperPresent && lowerPresent) stop();

      case Reversing:
        reverseBoth();
      case Idle:
        stopBoth();
    }

    SmartDashboard.putBoolean("Upper Ball Present", upperPresent);
    SmartDashboard.putBoolean("Lower Ball Present", lowerPresent);

    if (Constants.TUNING_MODE) {
      SmartDashboard.putString("Tower State", m_state.toString());
      SmartDashboard.putString("Upper Ball Alliance", upperSensor.getBallAlliance().toString());
      SmartDashboard.putString("Lower Ball Alliance", lowerSensor.getBallAlliance().toString());
      SmartDashboard.putString("Upper Ball Color", upperSensor.rawColor());
      SmartDashboard.putString("Lower Ball Color", lowerSensor.rawColor());
    }
  }

  /// -------- State Machine API -------- ///
  public void stop() {
    m_state = State.Idle;
  }

  public void reverse() {
    m_state = State.Reversing;
  }

  public void load() {
    m_state = State.Loading;
  }

  /** if false, load for firing. if true, send up to fire */
  public void feed(boolean ready) {
    m_state = State.Feeding;
    firing = ready;
  }

  /// -------- Motor Control -------- ///
  private void reverseBoth() {
    lowerMotor.set(ControlMode.PercentOutput, -kReversingSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, -kReversingSpeedUpper);
  }

  private void stopBoth() {
    lowerMotor.set(ControlMode.PercentOutput, 0.0);
    upperMotor.set(ControlMode.PercentOutput, 0.0);
  }

  private void loadBoth() {
    lowerMotor.set(ControlMode.PercentOutput, kLoadingSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, kLoadingSpeedUpper);
  }

  private void loadLower() {
    lowerMotor.set(ControlMode.PercentOutput, kLoadingSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, 0.0);
  }

  private void feedBoth() {
    lowerMotor.set(ControlMode.PercentOutput, kShootingSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, kShootingSpeedUpper);
  }

  private void feedUpper() {
    lowerMotor.set(ControlMode.PercentOutput, 0.0);
    upperMotor.set(ControlMode.PercentOutput, kShootingSpeedUpper);
  }

  /// -------- Color Sensors -------- ///
  /** Note: will return false if no ball present */
  public boolean upperBallOurs() {
    final var ballAlliance = upperSensor.getBallAlliance();
    return upperBallPresent() && (ballAlliance == ourAlliance)
        || (ballAlliance == Alliance.Invalid);
  }

  private boolean upperBallPresent() {
    return upperSensor.ballPresent();
  }

  private boolean lowerBallPresent() {
    return lowerSensor.ballPresent();
  }

  public void setAlliance(Alliance ours) {
    this.ourAlliance = ours;
  }
}
