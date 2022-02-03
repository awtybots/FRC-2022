// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TeleopDrive extends CommandBase {

  private final DrivetrainSubsystem m_Drive;
  private final XboxController m_Controller;

  public TeleopDrive(XboxController controller, DrivetrainSubsystem drive) {
    addRequirements(drive);
    this.m_Drive = drive;
    this.m_Controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Split Arcade
    // double forward = m_Controller.getLeftY();
    // double rotate = m_Controller.getRightX();
    // RC Car
    double forward = m_Controller.getRightTriggerAxis() - m_Controller.getLeftTriggerAxis();
    double rotate = m_Controller.getLeftX();
    m_Drive.drive(forward, rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
