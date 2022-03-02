package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Field;
import frc.robot.Constants.Shooter;
import frc.robot.commands.backup.Spit;
import frc.robot.subsystems.ColorSensorsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.math.ProjectileMotionSolver;
import frc.robot.util.math.ProjectileMotionSolver.CommonProjectiles.Sphere;
import frc.robot.util.math.Vector2;

/**
 * The goal is that this command runs the entire time. It constantly aims the turret according to
 * the projectile motion solver, and it shoots or spits balls automatically. It also handles the
 * tower logic, so if this is the command of choice, the intake command should not deal with the
 * tower.
 */
public class MovingShots extends CommandBase {
  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final ColorSensorsSubsystem colorSensorsSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final DrivetrainSubsystem drivetrainSubsystem;

  private final ProjectileMotionSolver projectileMotionSolver;

  public MovingShots(
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      DrivetrainSubsystem drivetrainSubsystem,
      ColorSensorsSubsystem colorSensorsSubsystem,
      LimelightSubsystem limelightSubsystem) {
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.colorSensorsSubsystem = colorSensorsSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(
        towerSubsystem,
        shooterSubsystem,
        turretSubsystem,
        limelightSubsystem); // drive and color sensor subsystems not requirements

    projectileMotionSolver =
        new ProjectileMotionSolver(
            Field.kBallMass,
            Sphere.frontalArea(Field.kBallRadius),
            Sphere.dragCoefficient,
            Shooter.kLaunchAngle);
  }

  @Override
  public void initialize() {
    limelightSubsystem.shootingMode();
  }

  private void executeShoot(boolean idling) {
    if (!limelightSubsystem.hasVisibleTarget()) {
      if (turretSubsystem.isAtTarget()) {
        turretSubsystem.turnBy(5.0); // seeking
      }
      return;
    }

    Vector2 goalDisplacement = limelightSubsystem.getGoalDisplacement();

    double visionTargetXOffset = limelightSubsystem.getTargetXOffset();
    double robotSpeed = drivetrainSubsystem.getSpeed();
    double driveToGoalAngle = visionTargetXOffset + turretSubsystem.getActualAngle();
    Vector2 robotVelocity = Vector2.fromPolar(robotSpeed, driveToGoalAngle);

    Vector2 optimalLaunchVelocity =
        projectileMotionSolver.getOptimalLaunchVelocityMoving(goalDisplacement, robotVelocity);
    if (optimalLaunchVelocity == null) {
      // ?
      return;
    }

    double launchVelocity = optimalLaunchVelocity.x; // meters per second
    double horizontalLaunchAngle = optimalLaunchVelocity.y; // degrees counterclockwise

    double launchRpm = launchVelocity / (Shooter.kFlywheelDiameter * Math.PI) * 60.0;

    turretSubsystem.turnBy(visionTargetXOffset - horizontalLaunchAngle);
    if(idling)
      shooterSubsystem.stop();
    else
      shooterSubsystem.shootRpm(launchRpm);
  }

  private void executeSpit() {
    turretSubsystem.turnTo(180.0);
    shooterSubsystem.shootRpm(Spit.kRpm);
  }

  @Override // TODO make sure intake can run at the same time
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

        if(colorSensorsSubsystem.isLowerBallPresent()) {
          towerSubsystem.stop();
        }
      }
    } else {
      executeShoot(true);
      towerSubsystem.intake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    towerSubsystem.stop();
    shooterSubsystem.stop();
    turretSubsystem.stop();
    limelightSubsystem.drivingMode();
  }
}
