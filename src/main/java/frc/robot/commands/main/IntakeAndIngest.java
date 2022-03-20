package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensorsSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;

/** comprehensive intake command */
public class IntakeAndIngest extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;
  private final TowerSubsystem towerSubsystem;
  private final ColorSensorsSubsystem colorSensorsSubsystem;

  public IntakeAndIngest(
      IntakeSubsystem intakeSubsystem,
      TowerSubsystem towerSubsystem,
      ColorSensorsSubsystem colorSensorsSubsystem) {
    addRequirements(
        intakeSubsystem, towerSubsystem); // note: ColorSensorsSubsystem is not a requirement

    this.intakeSubsystem = intakeSubsystem;
    this.towerSubsystem = towerSubsystem;
    this.colorSensorsSubsystem = colorSensorsSubsystem;
  }

  @Override
  public void initialize() {
    intakeSubsystem.start();
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
    intakeSubsystem.stop();
    towerSubsystem.stop();
  }
}