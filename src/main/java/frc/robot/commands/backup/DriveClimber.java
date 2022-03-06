// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.util.Controller;

public class DriveClimber extends CommandBase {

  private final ClimbSubsystem climbSubsystem;
  private final Controller controller;

  public DriveClimber(Controller controller, ClimbSubsystem climbSubsystem) {
    addRequirements(climbSubsystem);
    this.climbSubsystem = climbSubsystem;
    this.controller = controller;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double rate = controller.getRightTrigger() - controller.getLeftTrigger();
    climbSubsystem.drive(rate);
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
