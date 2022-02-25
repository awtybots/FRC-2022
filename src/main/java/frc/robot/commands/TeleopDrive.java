// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.math.Vector2;

public class TeleopDrive extends CommandBase {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final XboxController controller;

  public TeleopDrive(XboxController controller, DrivetrainSubsystem drive) {
    addRequirements(drive);
    this.drivetrainSubsystem = drive;
    this.controller = controller;
  }

  @SuppressWarnings("unused")
  private Vector2 splitArcadeDrive() {
    double forward = controller.getLeftY();
    double rotate = controller.getRightX();

    return new Vector2(forward, rotate);
  }

  private Vector2 gtaDrive() {
    double forward = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
    double rotate = controller.getLeftX();

    return new Vector2(forward, rotate);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Vector2 processedDriveInput = gtaDrive();
    drivetrainSubsystem.drive(processedDriveInput.x, processedDriveInput.y);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
