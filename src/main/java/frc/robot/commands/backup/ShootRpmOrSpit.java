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
  private boolean alreadySet = false;

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
    alreadySet = false;

    if (!limelightSubsystem.hasVisibleTarget()) {
      turretSubsystem.seek();
      return;
    }

    double deltaAngle = limelightSubsystem.cameraTargetAngleDelta();

    turretSubsystem.turnBy(deltaAngle);

    if (idling) shooterSubsystem.stop();
    else shooterSubsystem.shootRpm(this.rpm);
  }

  private void executeSpit() {
    if (!alreadySet) {
      alreadySet = true;
      if (limelightSubsystem.hasVisibleTarget()) {
        turretSubsystem.spitRelative(limelightSubsystem.cameraTargetAngleDelta());
      } else {
        turretSubsystem.spit();
      }
    }
    shooterSubsystem.spit();
  }

  @Override
  public void execute() {
    if (colorSensorsSubsystem.isUpperBallPresent()) {
      if (colorSensorsSubsystem.isUpperBallOurs()) {
        executeShoot(false);
      } else {
        executeSpit();
      }

      if (turretSubsystem.isAtTarget() && shooterSubsystem.isAtTarget()) {
        towerSubsystem.feedShooter();
      } else {
        towerSubsystem.stopUpper();

        if (colorSensorsSubsystem.isLowerBallPresent()) {
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
    turretSubsystem.stop();
    limelightSubsystem.drivingMode();
  }
}
