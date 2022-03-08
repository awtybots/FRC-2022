package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class ShootRpm extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;
  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final double rpm;

  public ShootRpm(double rpm, IntakeSubsystem intakeSubsystem, TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.rpm = rpm;

    addRequirements(intakeSubsystem, towerSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.shootRpm(this.rpm);
    intakeSubsystem.start();
  }

  @Override
  public void execute() {
    if (shooterSubsystem.isAtTarget()) {
      towerSubsystem.feedShooter();
    } else {
      towerSubsystem.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    towerSubsystem.stop();
    intakeSubsystem.stop();
  }
}
