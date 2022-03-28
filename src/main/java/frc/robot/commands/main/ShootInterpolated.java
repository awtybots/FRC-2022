package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.*;

public class ShootInterpolated extends CommandBase {
  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  public ShootInterpolated(
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem) {
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(towerSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.shootingMode();
    towerSubsystem.ingest();
  }

  @Override
  public void execute() {
    double goalDistance = limelightSubsystem.distToTarget();

    if (goalDistance == -1) {
      shooterSubsystem.stop();
    } else {
      double launchRpm = Shooter.shotMap.calculateShot(goalDistance);
      shooterSubsystem.shootRpm(launchRpm);
      towerSubsystem.feed(shooterSubsystem.isAtTarget());
    }
  }

  @Override
  public void end(boolean interrupted) {
    towerSubsystem.stop();
    shooterSubsystem.stop();
    limelightSubsystem.drivingMode();
  }
}
