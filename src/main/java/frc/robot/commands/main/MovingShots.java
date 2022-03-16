package frc.robot.commands.main;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Field;
import frc.robot.subsystems.*;
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

  private boolean alreadySet = false;

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
            ShooterSubsystem.kLaunchAngle);
  }

  @Override
  public void initialize() {
    limelightSubsystem.shootingMode();
  }

  private void executeShoot(boolean idling) {
    alreadySet = false;

    if (!limelightSubsystem.hasVisibleTarget()) {
      turretSubsystem.seek();
      shooterSubsystem.stop();
      return;
    }

    double goalDisplacement = limelightSubsystem.distToTarget();
    if (goalDisplacement == -1) {
      turretSubsystem.seek();
      shooterSubsystem.stop();
      return;
    }

    System.out.println(goalDisplacement); // TODO remove

    double visionTargetXOffset = limelightSubsystem.cameraTargetAngleDelta();
    double robotSpeed = drivetrainSubsystem.getSpeed();
    double driveToGoalAngle = visionTargetXOffset + turretSubsystem.getActualAngle();
    Vector2 robotVelocity = Vector2.fromPolar(robotSpeed, -driveToGoalAngle);

    Vector2 launchVelocityData =
        projectileMotionSolver.getOptimalLaunchVelocityMoving(
            new Vector2(goalDisplacement, Field.kGoalHeight), robotVelocity);

    if (launchVelocityData == null) {
      shooterSubsystem.stop();
      return;
    }

    double launchVelocity = launchVelocityData.x; // meters per second
    double horizontalLaunchAngle = launchVelocityData.y; // degrees clockwise

    double launchRpm = ShooterSubsystem.ballVelocityToFlywheelRpm(launchVelocity);

    turretSubsystem.turnBy(visionTargetXOffset + horizontalLaunchAngle);

    if (idling) {
      shooterSubsystem.stop();
    } else {
      // shooterSubsystem.shootRpm(launchRpm);
      SmartDashboard.putNumber("moving shots rpm", launchRpm);
    }
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
