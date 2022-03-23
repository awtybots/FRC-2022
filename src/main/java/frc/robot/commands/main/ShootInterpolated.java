package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.*;

public class ShootInterpolated extends CommandBase {
  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final ColorSensorsSubsystem colorSensorsSubsystem;

  public ShootInterpolated(
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem,
      ColorSensorsSubsystem colorSensorsSubsystem) {
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.colorSensorsSubsystem = colorSensorsSubsystem;

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

    double launchRpm = Shooter.shotMap.calculateShot(goalDistance);
    shooterSubsystem.shootRpm(launchRpm);

    if (shooterSubsystem.isAtTarget()) {
      if(colorSensorsSubsystem.isUpperBallPresent()) {
        towerSubsystem.feedShooter1();
      } else {
        towerSubsystem.feedShooter2();
      }
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
