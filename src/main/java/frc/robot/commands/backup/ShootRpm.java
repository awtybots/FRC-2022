package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class ShootRpm extends CommandBase {
  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final double rpm;

  public ShootRpm(double rpm, TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem) {
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.rpm = rpm;

    addRequirements(towerSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.shootRpm(this.rpm);
  }

  @Override
  public void execute() {
    if (shooterSubsystem.isRpmAtTarget()) {
      towerSubsystem.feedShooter();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    towerSubsystem.stop();
  }
}
