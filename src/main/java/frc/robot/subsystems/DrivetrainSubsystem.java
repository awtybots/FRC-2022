// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain;

public class DrivetrainSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftFront, leftBack, rightFront, rightBack;
  private final WPI_TalonFX[] motors;

  private final MotorControllerGroup leftDrive, rightDrive;
  private final DifferentialDrive drivetrain;

  public DrivetrainSubsystem() {
    leftFront = new WPI_TalonFX(Drivetrain.kLeftFront);
    leftBack = new WPI_TalonFX(Drivetrain.kLeftBack);
    rightFront = new WPI_TalonFX(Drivetrain.kRightFront);
    rightBack = new WPI_TalonFX(Drivetrain.kRightBack);

    motors = new WPI_TalonFX[] {leftFront, leftBack, rightFront, rightBack};

    configMotors();

    leftDrive = new MotorControllerGroup(leftFront, leftBack);
    rightDrive = new MotorControllerGroup(rightFront, rightBack);

    drivetrain = new DifferentialDrive(leftDrive, rightDrive);
  }

  private void configMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.voltageCompSaturation = 12.0;
    config.openloopRamp = 0.1;

    for (WPI_TalonFX motor : motors) {
      motor.configAllSettings(config);
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      motor.setSelectedSensorPosition(0.0);
      motor.setNeutralMode(NeutralMode.Brake);
    }

    rightFront.setInverted(TalonFXInvertType.Clockwise);
    leftBack.setInverted(TalonFXInvertType.Clockwise);
  }

  public void drive(double forward, double rotate) {
    // We use this because it behaves like a differential in a car.
    // The forward velocity of the robot is maintained through a turn, only
    // adjusting the turn rate.
    // in short: leftspeed = forward + rotate; rightspeed = forward - rotate
    drivetrain.curvatureDrive(forward, rotate, true);

    // If desired, during a turn, the forward speed of the robot can be reduced
    // by only adjusting one wheel speed. For example, during a left turn, the
    // right wheel speed remains at the straightline speed while the left wheel
    // slows down to make the turn. this reduces the forward speed of the robot.
    // In short, on a left turn: leftspeed = forward - rotate; rightspeed = forward
    // This behaviour can be enabled by uncommenting the line below.
    // drivetrain.arcadeDrive(forward, rotate, true);
  }
}
