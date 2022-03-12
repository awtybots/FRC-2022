// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;
import frc.robot.util.math.Convert;
import frc.robot.util.math.Convert.Encoder;

public class ClimbSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftMotor;
  private final WPI_TalonFX rightMotor;

  private final WPI_TalonFX[] motors;

  private static final double kWinchDiameter = Convert.inchesToMeters(1.0);
  private static final double kGearRatio = 1.0 / 5.0 / 5.0 * 34.0 / 44.0;

  private static final double kMaxPercentOutput = 1.0;
  private static final double kRamp = 0.2;

  public ClimbSubsystem() {
    leftMotor = new WPI_TalonFX(Climber.kLeftMotorCanId);
    rightMotor = new WPI_TalonFX(Climber.kRightMotorCanId);
    motors = new WPI_TalonFX[] {rightMotor}; // TODO add back left one

    configMotors();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("CL - position", Convert.metersToInches(getPosition()));
  }

  private double getPosition() {
    double sum = 0.0;
    for (WPI_TalonFX motor : motors) {
      sum +=
          Convert.encoderPosToDistance(
              motor.getSelectedSensorPosition(),
              kGearRatio,
              kWinchDiameter,
              Encoder.TalonFXIntegrated);
    }
    return sum / motors.length;
  }

  private void configMotors() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.setInverted(TalonFXInvertType.Clockwise);

    for (WPI_TalonFX motor : motors) {
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      motor.setSelectedSensorPosition(0.0);

      motor.setNeutralMode(NeutralMode.Brake);

      motor.configOpenloopRamp(kRamp);
      motor.configClosedloopRamp(kRamp);
      motor.configPeakOutputForward(kMaxPercentOutput);
      motor.configPeakOutputReverse(-kMaxPercentOutput);
      motor.configClosedLoopPeakOutput(0, kMaxPercentOutput);
    }
  }

  public void drive(double pct) {
    for (WPI_TalonFX motor : motors) motor.set(ControlMode.PercentOutput, pct * kMaxPercentOutput);
  }

  public void stop() {
    for (WPI_TalonFX motor : motors) motor.set(ControlMode.PercentOutput, 0.0);
  }
}
