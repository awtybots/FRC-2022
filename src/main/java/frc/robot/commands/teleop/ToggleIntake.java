package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntake extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;

  public ToggleIntake(IntakeSubsystem intakeSubsystem) {
    addRequirements(intakeSubsystem);

    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    intakeSubsystem.togglePistons(true);
    intakeSubsystem.toggleWheels(true);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.togglePistons(false);
    intakeSubsystem.toggleWheels(false);
  }
}
