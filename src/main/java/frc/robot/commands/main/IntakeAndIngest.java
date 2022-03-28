package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;

/** comprehensive intake command */
public class IntakeAndIngest extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;
  private final TowerSubsystem towerSubsystem;

  public IntakeAndIngest(IntakeSubsystem intakeSubsystem, TowerSubsystem towerSubsystem) {
    addRequirements(intakeSubsystem, towerSubsystem);

    this.intakeSubsystem = intakeSubsystem;
    this.towerSubsystem = towerSubsystem;
  }

  @Override
  public void initialize() {
    intakeSubsystem.start();
    towerSubsystem.ingest();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    towerSubsystem.stop();
  }
}
