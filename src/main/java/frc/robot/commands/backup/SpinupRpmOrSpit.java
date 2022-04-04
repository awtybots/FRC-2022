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

  private void executeShoot(boolean idling) {
    turret.trackTarget(limelight.hasVisibleTarget(), limelight.angleToTarget());

    if (idling) shooter.stop();
    else shooter.shootRpm(this.rpm);
  }

  private void executeSpit() {
    turret.spit(limelight.hasVisibleTarget(), limelight.angleToTarget());
    shooter.spit();
  }

  @Override
  public void execute() {
    // TODO port
    // if (colorSensorsSubsystem.isUpperBallPresent()) {
    //   if (colorSensorsSubsystem.isUpperBallOurs()) {
    //     executeShoot(false);
    //   } else {
    //     executeSpit();
    //   }

    //   if (turretSubsystem.isAtTarget() && shooterSubsystem.isAtTarget()) {
    //     if (colorSensorsSubsystem.isUpperBallPresent()) {
    //       towerSubsystem.feedFromUpper();
    //     } else {
    //       towerSubsystem.feedFromLower();
    //     }
    //   } else {
    //     towerSubsystem.stopUpper();

    //     if (colorSensorsSubsystem.isLowerBallPresent()) {
    //       towerSubsystem.stop();
    //     }
    //   }
    // } else {
    //   towerSubsystem.intake();
    //   executeShoot(true);
    // }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    tower.stop();
    turret.idle();
    limelight.enableDrivingMode();
  }
}
