package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class SpinupRpmOrSpit extends CommandBase {
  private final TowerSubsystem tower;
  private final ShooterSubsystem shooter;
  private final TurretSubsystem turret;
  private final LimelightSubsystem limelight;

  private final double rpm;

  public SpinupRpmOrSpit(
      double rpm,
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      LimelightSubsystem limelightSubsystem) {

    this.tower = towerSubsystem;
    this.shooter = shooterSubsystem;
    this.turret = turretSubsystem;
    this.limelight = limelightSubsystem;

    this.rpm = rpm;

    addRequirements(towerSubsystem, shooterSubsystem, turretSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelight.enableShootingMode();
  }

  private void aimForTarget() {
    turret.trackTarget(limelight.hasVisibleTarget(), limelight.angleToTarget());
  }

  private void spinup() {
    shooter.shootRpm(this.rpm);
  }

  private void spinupToSpit() {
    shooter.spit();
  }

  private void aimToSpit() {
    turret.spit(limelight.hasVisibleTarget(), limelight.angleToTarget());
  }

  @Override
  public void execute() {
    if (!tower.upperBallPresent()) {
      aimForTarget();
      shooter.stop();
    } else if (tower.upperBallOurs()) {
      aimForTarget();
      spinup();
    } else {
      aimToSpit();
      spinupToSpit();
    }

    tower.feed(shooter.isAtTarget() && turret.isAtTarget());
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    tower.stop();
    turret.idle();
    limelight.enableDrivingMode();
  }
}
