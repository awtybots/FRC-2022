package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.*;

public class ShootInterpolatedOrSpit extends CommandBase {

  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final ColorSensorsSubsystem colorSensorsSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  public ShootInterpolatedOrSpit(
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      ColorSensorsSubsystem colorSensorsSubsystem,
      LimelightSubsystem limelightSubsystem) {

    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.colorSensorsSubsystem = colorSensorsSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(
        towerSubsystem,
        shooterSubsystem,
        turretSubsystem,
        limelightSubsystem); // drive and color sensor subsystems not requirements
  }

  @Override
  public void initialize() {
    limelightSubsystem.shootingMode();
  }

  private void executeShoot() {
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

    double launchRpm = Shooter.shotMap.calculateShot(goalDisplacement);
    shooterSubsystem.shootRpm(launchRpm);
  }

  private void executeSpit() {
    turretSubsystem.spit(
        limelightSubsystem.hasVisibleTarget(), limelightSubsystem.cameraTargetAngleDelta());
    shooterSubsystem.spit();
  }

  @Override
  public void execute() {
    if (colorSensorsSubsystem.isUpperBallPresent()) {
      if (colorSensorsSubsystem.isUpperBallOurs()) {
        executeShoot();
      } else {
        executeSpit();
      }

      if (turretSubsystem.isAtTarget() && shooterSubsystem.isAtTarget()) {
        if (colorSensorsSubsystem.isUpperBallPresent()) {
          towerSubsystem.feedShooter1();
        } else {
          towerSubsystem.feedShooter2();
        }
      } else {
        towerSubsystem.stopUpper();

        if (colorSensorsSubsystem.isLowerBallPresent()) {
          towerSubsystem.stop();
        }
      }
    } else {
      executeShoot();
      towerSubsystem.intake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    towerSubsystem.stop();
    shooterSubsystem.stop();
    turretSubsystem.idle();
    limelightSubsystem.drivingMode();
  }
}
