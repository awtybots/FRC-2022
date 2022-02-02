// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain;

public class DrivetrainSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftFront, leftBack, rightFront, rightBack;
  private final MotorControllerGroup leftDrive, rightDrive;
  private final DifferentialDrive drivetrain;

  public DrivetrainSubsystem() {
    leftFront = new WPI_TalonFX(0);
    leftBack = new WPI_TalonFX(1);
    rightFront = new WPI_TalonFX(2);
    rightBack = new WPI_TalonFX(3);

    leftFront.configAllSettings(Drivetrain.motorConfig());
    rightFront.configAllSettings(Drivetrain.motorConfig());
    leftBack.configAllSettings(Drivetrain.motorConfig());
    rightBack.configAllSettings(Drivetrain.motorConfig());

    leftDrive = new MotorControllerGroup(leftFront, leftBack);
    rightDrive = new MotorControllerGroup(rightFront, rightBack);
    rightDrive.setInverted(true);

    drivetrain = new DifferentialDrive(leftDrive, rightDrive);
  }

  public void arcadeDrive(double forward, double rotate) {
    drivetrain.arcadeDrive(forward, rotate, true);
  }
}
