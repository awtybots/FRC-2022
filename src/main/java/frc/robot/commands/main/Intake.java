package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.IntakeSubsystem;

/** This only runs the intake, not tower. This is what we use if we do moving shots. */
public class Intake extends StartEndCommand {
  public Intake(IntakeSubsystem intakeSubsystem) {
    super(intakeSubsystem::start, intakeSubsystem::stop, intakeSubsystem);
  }
}
