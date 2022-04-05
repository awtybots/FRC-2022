// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.util.Controller;

public class DriveClimber extends CommandBase {

  private final ClimbSubsystem climber;
  private final Controller controller;

  public DriveClimber(Controller controller, ClimbSubsystem climber) {
    addRequirements(climber);
    this.climber = climber;
    this.controller = controller;
  }

  @Override
  public void execute() {
    double rate = controller.getRightTrigger() - controller.getLeftTrigger();
    climber.drive(rate);
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }
}
