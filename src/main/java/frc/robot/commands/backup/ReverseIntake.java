package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseIntake extends StartEndCommand {

  public ReverseIntake(IntakeSubsystem intakeSubsystem) {
    super(intakeSubsystem::startReversed, intakeSubsystem::stop, intakeSubsystem);
  }
}
