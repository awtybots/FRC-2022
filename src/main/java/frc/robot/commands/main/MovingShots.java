package frc.robot.commands.main;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Field;
import frc.robot.subsystems.*;
import frc.util.math.ProjectileMotionSolver;
import frc.util.math.ProjectileMotionSolver.CommonProjectiles.Sphere;
import frc.util.math.Vector2;

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
            ShooterSubsystem.kLaunchAngle);
  }

  @Override
  public void initialize() {
    limelightSubsystem.shootingMode();
  }

  private void executeShoot(boolean idling) {
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
    double robotSpeed = drivetrainSubsystem.getSpeed();
    double driveToGoalAngle = visionTargetXOffset + turretSubsystem.getCurrentAngle();
    Vector2 robotVelocity = Vector2.fromPolar(robotSpeed, -driveToGoalAngle);

    Vector2 launchVelocityData =
        projectileMotionSolver.getOptimalLaunchVelocityMoving(
            new Vector2(
                goalDisplacement,
                Field.kGoalHeight
                    - Constants.Limelight.kMountingHeight
                    - Constants.Limelight.kShooterOffset.y),
            robotVelocity);

    if (launchVelocityData == null) {
      shooterSubsystem.stop();
      return;
    }

    double launchVelocity = launchVelocityData.x; // meters per second
    double horizontalLaunchAngle = launchVelocityData.y; // degrees clockwise

    double launchRpm = ShooterSubsystem.ballVelocityToFlywheelRpm(launchVelocity);

    turretSubsystem.trackTarget(true, visionTargetXOffset + horizontalLaunchAngle);

    if (idling) {
      shooterSubsystem.stop();
    } else {
      // shooterSubsystem.shootRpm(launchRpm);
      SmartDashboard.putNumber("moving shots rpm", launchRpm);
    }
  }

  private void executeSpit() {
    turretSubsystem.spit(
        limelightSubsystem.hasVisibleTarget(), limelightSubsystem.cameraTargetAngleDelta());
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
      executeShoot(true);
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
