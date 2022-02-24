// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TeleopDrive extends CommandBase {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final XboxController controller;

  public TeleopDrive(XboxController controller, DrivetrainSubsystem drive) {
    addRequirements(drive);
    this.drivetrainSubsystem = drive;
    this.controller = controller;
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Split Arcade
    // double forward = controller.getLeftY();
    // double rotate = controller.getRightX();
    // RC Car
    double forward = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
    double rotate = controller.getLeftX();
    drivetrainSubsystem.drive(forward, rotate);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
