package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TowerSubsystem;

/** comprehensive intake command */
public class IntakeAndIngest extends CommandBase {
  private final TowerSubsystem tower;

  public IntakeAndIngest(TowerSubsystem tower) {
    addRequirements(tower);

    this.tower = tower;
  }

  @Override
  public void initialize() {
    tower.intake();
  }

  @Override
  public boolean isFinished() {
    return tower.isFull();
  }

  @Override
  public void end(boolean interrupted) {
    tower.stopIntaking();
  }
}
