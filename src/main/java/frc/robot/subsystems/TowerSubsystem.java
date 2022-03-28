// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorSensors;
import frc.robot.Constants.Tower;

public class TowerSubsystem extends SubsystemBase {

  private State m_state = State.Idle;
  private boolean firing = false;

  private Alliance ourAlliance;
  private final ColorSensor lowerSensor;
  private final ColorSensor upperSensor;

  private final WPI_TalonSRX upperMotor, lowerMotor;

  private static final double kIntakingSpeedLower = 0.4;
  private static final double kIntakingSpeedUpper = 0.3;

  private static final double kReversingSpeedLower = 0.5;
  private static final double kReversingSpeedUpper = 0.4;

  private static final double kShootingSpeedLower = 0.3;
  private static final double kShootingSpeedUpper = 0.75;

  public TowerSubsystem() {
    upperSensor = new ColorSensor(ColorSensors.kUpperSensorPort);
    lowerSensor = new ColorSensor(ColorSensors.kLowerSensorPort);

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
      case Idle:
        stopBoth();

      case Reversing:
        reverseBoth();

      case Feeding:
        if (!firing) {
          load();
        } else {
          if (upperPresent) {
            feedFromUpper();
          } else feedFromLower();
        }
      case Loading:
        if (!upperPresent) intake();
        if (upperPresent && !lowerPresent) intakeLowerOnly();
        if (upperPresent && lowerPresent) stopBoth();
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

  public void feed(boolean ready) {
    m_state = State.Feeding;
    firing = ready;
  }

  /// -------- Motor Control -------- ///
  private void intake() {
    lowerMotor.set(ControlMode.PercentOutput, kIntakingSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, kIntakingSpeedUpper);
  }

  private void intakeLowerOnly() {
    lowerMotor.set(ControlMode.PercentOutput, kIntakingSpeedLower);
  }

  private void reverseBoth() {
    lowerMotor.set(ControlMode.PercentOutput, -kReversingSpeedLower);
    upperMotor.set(ControlMode.PercentOutput, -kReversingSpeedUpper);
  }

  /** stops lower tower, runs upper tower */
  private void feedFromUpper() {
    lowerMotor.set(ControlMode.PercentOutput, 0.0);
    upperMotor.set(ControlMode.PercentOutput, kShootingSpeedUpper);
  }

  /** runs upper and lower tower */
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

  /// -------- Color Sensors -------- ///
  public boolean upperBallOurs() {
    return upperSensor.getBallAlliance() == ourAlliance;
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

  private class ColorSensor {

    private final int minimumDistance = 250;
    private final double minColorConfidence = 0.90;

    // Tune these at each field if you want to know the color of the balls
    private final Color Red = new Color(0.41, 0.41, 0.18);
    private final Color Blue = new Color(0.17, 0.41, 0.43);

    private final ColorSensorV3 sensor;
    private final ColorMatch colorMatch = new ColorMatch();

    public ColorSensor(I2C.Port port) {
      sensor = new ColorSensorV3(port);
      sensor.configureColorSensor(
          ColorSensorResolution.kColorSensorRes17bit,
          ColorSensorMeasurementRate.kColorRate25ms,
          GainFactor.kGain3x);
      sensor.configureProximitySensor(
          ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate6ms);

      sensor.configureProximitySensor(
          ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate6ms);

      sensor.configureColorSensor(
          ColorSensorResolution.kColorSensorRes17bit,
          ColorSensorMeasurementRate.kColorRate50ms,
          GainFactor.kGain3x);

      colorMatch.addColorMatch(Red);
      colorMatch.addColorMatch(Blue);
      colorMatch.setConfidenceThreshold(minColorConfidence);
    }

    public boolean ballPresent() {
      return sensor.getProximity() > minimumDistance;
    }

    public Alliance getBallAlliance() {
      Color detectedColor = sensor.getColor();
      ColorMatchResult match = colorMatch.matchColor(detectedColor);

      if (match != null) {
        if (match.color == Red) return Alliance.Red;
        if (match.color == Blue) return Alliance.Blue;
      }

      return Alliance.Invalid;
    }

    public String rawColor() {
      return rgbToString(sensor.getColor());
    }

    private String rgbToString(Color c) {
      return String.format("RGB(%.2f, %.2f, %.2f)", c.red, c.green, c.blue);
    }
  }
}
