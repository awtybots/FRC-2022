// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

public class ClimbSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftMotor;
  private final WPI_TalonFX rightMotor;

  public ClimbSubsystem() {
    leftMotor = new WPI_TalonFX(Climber.kLeftMotor);
    rightMotor = new WPI_TalonFX(Climber.kRightMotor);

    configMotors();
  }

  private void configMotors() {
    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightMotor.setInverted(TalonFXInvertType.Clockwise);
  }
}
