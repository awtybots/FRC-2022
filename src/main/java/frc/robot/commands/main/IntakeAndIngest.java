package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TowerV2Subsystem;

/** comprehensive intake command */
public class IntakeAndIngest extends CommandBase {
  private final TowerV2Subsystem towerSubsystem;

  public IntakeAndIngest(TowerV2Subsystem towerSubsystem) {
    addRequirements(towerSubsystem);

    this.towerSubsystem = towerSubsystem;
  }

  @Override
  public void initialize() {
    towerSubsystem.intake();
  }

  @Override
  public boolean isFinished() {
    return towerSubsystem.isFull();
  }

  @Override
  public void end(boolean interrupted) {
    towerSubsystem.stopIntaking();
  }
}
