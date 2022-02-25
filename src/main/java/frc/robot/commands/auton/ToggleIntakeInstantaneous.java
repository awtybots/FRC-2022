package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntakeInstantaneous extends InstantCommand {

  public ToggleIntakeInstantaneous(IntakeSubsystem intakeSubsystem, boolean on) {
    super(
        () -> {
          intakeSubsystem.togglePistons(on);
          intakeSubsystem.toggleWheels(on);
        });
    addRequirements(intakeSubsystem);
  }
}
