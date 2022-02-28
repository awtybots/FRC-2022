package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class IntakeAndShoot extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;
  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final double rpm;

  public IntakeAndShoot(double rpm,
      IntakeSubsystem intakeSubsystem,
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem) {
    addRequirements(
        intakeSubsystem, towerSubsystem, shooterSubsystem);

    this.intakeSubsystem = intakeSubsystem;
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.rpm = rpm;
  }

  @Override
  public void initialize() {
    intakeSubsystem.start();
    towerSubsystem.feedShooter();
    shooterSubsystem.shootRpm(rpm);
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    towerSubsystem.stop();
    shooterSubsystem.stop();
  }
}
