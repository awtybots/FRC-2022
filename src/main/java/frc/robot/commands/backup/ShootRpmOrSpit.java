package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootRpmOrSpit extends CommandBase {
  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  private final double rpm;

  public ShootRpmOrSpit(
      double rpm,
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      LimelightSubsystem limelightSubsystem) {

    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    this.rpm = rpm;

    addRequirements(shooterSubsystem, turretSubsystem, limelightSubsystem); // Tower does arbitration
  }

  @Override
  public void initialize() {
    limelightSubsystem.shootingMode();
    towerSubsystem.claim();
    towerSubsystem.load();
  }

  private void aimForShooting() {
    turretSubsystem.trackTarget(
        limelightSubsystem.hasVisibleTarget(), limelightSubsystem.cameraTargetAngleDelta());
  }

  private void aimForSpitting() {
    turretSubsystem.spit(
        limelightSubsystem.hasVisibleTarget(), limelightSubsystem.cameraTargetAngleDelta());
  }

  @Override
  public void execute() {
    if (towerSubsystem.upperBallOurs()) {
      aimForShooting();
      shooterSubsystem.shootRpm(this.rpm);
    } else {
      aimForSpitting();
      shooterSubsystem.spit();
    }

    towerSubsystem.feed(turretSubsystem.isAtTarget() && shooterSubsystem.isAtTarget());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    towerSubsystem.stop();
    towerSubsystem.free();
    turretSubsystem.idle();
    limelightSubsystem.drivingMode();
  }
}
