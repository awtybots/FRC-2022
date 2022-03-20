package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.util.math.ShotMap;

public class ShootInterpolated extends CommandBase {
  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  private final ShotMap iMap;

  public ShootInterpolated(
      ShotMap sMap,
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem) {
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.iMap = sMap;

    addRequirements(towerSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.shootingMode();
  }

  @Override
  public void execute() {
    if (!limelightSubsystem.hasVisibleTarget()) {
      shooterSubsystem.stop();
      return;
    }

    double goalDistance = limelightSubsystem.distToTarget();
    if (goalDistance == -1) {
      shooterSubsystem.stop();
      return;
    }

    double launchRpm = iMap.calculateShot(goalDistance);
    shooterSubsystem.shootRpm(launchRpm);

    if (shooterSubsystem.isAtTarget()) {
      towerSubsystem.feedShooter();
    } else {
      towerSubsystem.stopUpper();
    }
  }

  @Override
  public void end(boolean interrupted) {
    towerSubsystem.stop();
    shooterSubsystem.stop();
    limelightSubsystem.drivingMode();
  }
}
