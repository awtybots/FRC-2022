package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.*;

public class ShootInterpolated extends CommandBase {
  private final TowerSubsystem tower;
  private final ShooterSubsystem shooter;
  private final LimelightSubsystem limelight;

  public ShootInterpolated(
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem) {
    this.tower = towerSubsystem;
    this.shooter = shooterSubsystem;
    this.limelight = limelightSubsystem;

    addRequirements(towerSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    limelight.enableShootingMode();
  }

  @Override
  public void execute() {
    if (!limelight.hasVisibleTarget()) {
      shooter.stop();
      return;
    }

    double goalDistance = limelight.distToTarget();
    if (goalDistance == -1) {
      shooter.stop();
      return;
    }

    double launchRpm = Shooter.shotMap.calculateShot(goalDistance);
    shooter.shootRpm(launchRpm);

    tower.feed(shooter.isAtTarget());
  }

  @Override
  public void end(boolean interrupted) {
    tower.stop();
    shooter.stop();
    limelight.enableDrivingMode();
  }
}
