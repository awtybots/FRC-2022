package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class ShootRpmNoIntake extends CommandBase {

  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final double rpm;

  public ShootRpmNoIntake(double rpm, TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem) {
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
  }
}
