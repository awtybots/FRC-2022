package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;

/** comprehensive intake command */
public class IntakeAndIngest extends CommandBase {
  private final IntakeSubsystem intake;
  private final TowerSubsystem tower;

  public IntakeAndIngest(IntakeSubsystem intakeSubsystem, TowerSubsystem towerSubsystem) {

    addRequirements(intakeSubsystem); // tower does arbitration, scheduler isn't involved

    this.intake = intakeSubsystem;
    this.tower = towerSubsystem;
  }

  @Override
  public void initialize() {
    intake.start();
    if (tower.available())
      tower.load();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    if (tower.available()) tower.stop();
  }
}
