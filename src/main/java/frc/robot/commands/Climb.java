package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class Climb extends CommandBase {

  private final ClimbSubsystem climbSubsystem;

  public Climb(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void initialize() {
    // TODO write climbing code
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}
}
