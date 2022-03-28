package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensorsSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootRpmOrSpit extends CommandBase {
  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final ColorSensorsSubsystem colorSensorsSubsystem;

  private final double rpm;

  public ShootRpmOrSpit(
      double rpm,
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      LimelightSubsystem limelightSubsystem,
      ColorSensorsSubsystem colorSensorsSubsystem) {

    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.colorSensorsSubsystem = colorSensorsSubsystem;

    this.rpm = rpm;

    addRequirements(towerSubsystem, shooterSubsystem, turretSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.shootingMode();
  }

  private void executeShoot(boolean idling) {
    turretSubsystem.trackTarget(
        limelightSubsystem.hasVisibleTarget(), limelightSubsystem.cameraTargetAngleDelta());

    if (idling) shooterSubsystem.stop();
    else shooterSubsystem.shootRpm(this.rpm);
  }

  private void executeSpit() {
    turretSubsystem.spit(
        limelightSubsystem.hasVisibleTarget(), limelightSubsystem.cameraTargetAngleDelta());
    shooterSubsystem.spit();
  }

  @Override
  public void execute() {
    if (colorSensorsSubsystem.upperBallPresent()) {
      if (colorSensorsSubsystem.upperBallOurs()) {
        executeShoot(false);
      } else {
        executeSpit();
      }

      if (turretSubsystem.isAtTarget() && shooterSubsystem.isAtTarget()) {
        if (colorSensorsSubsystem.upperBallPresent()) {
          towerSubsystem.feedFromUpper();
        } else {
          towerSubsystem.feedFromLower();
        }
      } else {
        towerSubsystem.stopUpper();

        if (colorSensorsSubsystem.lowerBallPresent()) {
          towerSubsystem.stop();
        }
      }
    } else {
      towerSubsystem.intake();
      executeShoot(true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    towerSubsystem.stop();
    turretSubsystem.idle();
    limelightSubsystem.drivingMode();
  }
}
