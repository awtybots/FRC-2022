package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensorsSubsystem;
import frc.robot.subsystems.TowerSubsystem;

/** comprehensive intake command */
public class IntakeAndIngest extends CommandBase {
  private final TowerSubsystem towerSubsystem;
  private final ColorSensorsSubsystem colorSensorsSubsystem;

  public IntakeAndIngest(
      TowerSubsystem towerSubsystem, ColorSensorsSubsystem colorSensorsSubsystem) {
    addRequirements(towerSubsystem);

    this.towerSubsystem = towerSubsystem;
    this.colorSensorsSubsystem = colorSensorsSubsystem;
  }

  @Override
  public void initialize() {
    towerSubsystem.intake();
  }

  @Override
  public void execute() {
    if (colorSensorsSubsystem.isUpperBallPresent()) {
      towerSubsystem.stopUpper();

      if (colorSensorsSubsystem.isLowerBallPresent()) {
        cancel();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    towerSubsystem.stop();
  }
}
