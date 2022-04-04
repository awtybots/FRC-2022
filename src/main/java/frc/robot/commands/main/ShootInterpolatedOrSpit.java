package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.*;

public class ShootInterpolatedOrSpit extends CommandBase {

  private final TowerSubsystem tower;
  private final ShooterSubsystem shooter;
  private final TurretSubsystem turret;
  private final LimelightSubsystem limelight;

  public ShootInterpolatedOrSpit(
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      LimelightSubsystem limelightSubsystem) {

    this.tower = towerSubsystem;
    this.shooter = shooterSubsystem;
    this.turret = turretSubsystem;
    this.limelight = limelightSubsystem;

    addRequirements(
        towerSubsystem,
        shooterSubsystem,
        turretSubsystem,
        limelightSubsystem); // drive subsystem not requirements
  }

  @Override
  public void initialize() {
    limelight.enableShootingMode();
  }

  private void executeShoot() {
    if (!limelight.hasVisibleTarget()) {
      turret.trackTarget(false);
      shooter.stop();
      return;
    }

    double goalDisplacement = limelight.distToTarget();
    if (goalDisplacement == -1) {
      turret.trackTarget(false);
      shooter.stop();
      return;
    }

    double visionTargetXOffset = limelight.angleToTarget();
    turret.trackTarget(true, visionTargetXOffset);

    double launchRpm = Shooter.shotMap.calculateShot(goalDisplacement);
    shooter.shootRpm(launchRpm);
  }

  private void executeSpit() {
    turret.spit(limelight.hasVisibleTarget(), limelight.angleToTarget());
    shooter.spit();
  }

  @Override
  public void execute() {
    if (!tower.upperBallPresent()) {
      executeShoot();
    } else if (tower.upperBallOurs()) {
      executeShoot();
    } else {
      executeSpit();
    }

    tower.feed(shooter.isAtTarget() && turret.isAtTarget());
  }

  @Override
  public void end(boolean interrupted) {
    tower.stop();
    shooter.stop();
    turret.idle();
    limelight.enableDrivingMode();
  }
}
