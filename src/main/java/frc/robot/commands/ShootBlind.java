package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class ShootBlind extends CommandBase {
  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final double speed;

  public ShootBlind(
      double speed, TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem) {
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    this.speed = speed;

    addRequirements(towerSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.percentShoot(speed); // ! FIXME write shooting code
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    towerSubsystem.stop();
  }
}
