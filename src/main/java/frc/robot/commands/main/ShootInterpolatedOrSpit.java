package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.*;

public class ShootInterpolatedOrSpit extends CommandBase {

  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private double launchRpm;

  public ShootInterpolatedOrSpit(
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      LimelightSubsystem limelightSubsystem) {

    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(towerSubsystem, shooterSubsystem, turretSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.shootingMode();
    towerSubsystem.claim();
  }

  private void aimForShooting() {
    if (!limelightSubsystem.hasVisibleTarget()) {
      turretSubsystem.trackTarget(false);
      shooterSubsystem.stop();
      return;
    }

    double goalDisplacement = limelightSubsystem.distToTarget();
    if (goalDisplacement == -1) {
      turretSubsystem.trackTarget(false);
      shooterSubsystem.stop();
      return;
    }

    double visionTargetXOffset = limelightSubsystem.cameraTargetAngleDelta();
    turretSubsystem.trackTarget(true, visionTargetXOffset);

    launchRpm = Shooter.shotMap.calculateShot(goalDisplacement);
  }

  private void aimForSpitting() {
    turretSubsystem.spit(
        limelightSubsystem.hasVisibleTarget(), limelightSubsystem.cameraTargetAngleDelta());
  }

  @Override
  public void execute() {
    if (towerSubsystem.upperBallOurs()) {
      aimForShooting();
      shooterSubsystem.shootRpm(launchRpm);
    } else {
      aimForSpitting();
      shooterSubsystem.spit();
    }

    towerSubsystem.feed(turretSubsystem.isAtTarget() && shooterSubsystem.isAtTarget());
  }

  @Override
  public void end(boolean interrupted) {
    towerSubsystem.stop();
    shooterSubsystem.stop();
    towerSubsystem.free();
    turretSubsystem.idle();
    limelightSubsystem.drivingMode();
  }
}
