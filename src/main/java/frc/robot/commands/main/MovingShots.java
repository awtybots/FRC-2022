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
  private final TowerSubsystem tower;
  private final ShooterSubsystem shooter;
  private final TurretSubsystem turret;
  private final LimelightSubsystem limelight;
  private final DrivetrainSubsystem drivetrain;

  private final ProjectileMotionSolver projectileMotionSolver;

  public MovingShots(
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      DrivetrainSubsystem drivetrainSubsystem,
      LimelightSubsystem limelightSubsystem) {
    this.tower = towerSubsystem;
    this.shooter = shooterSubsystem;
    this.turret = turretSubsystem;
    this.drivetrain = drivetrainSubsystem;
    this.limelight = limelightSubsystem;

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
    limelight.enableShootingMode();
  }

  private void executeShoot(boolean idling) {
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
    double robotSpeed = drivetrain.getSpeed();
    double driveToGoalAngle = visionTargetXOffset + turret.getCurrentAngle();
    Vector2 robotVelocity = Vector2.fromPolar(robotSpeed, -driveToGoalAngle);

    Vector2 launchVelocityData =
        projectileMotionSolver.getOptimalLaunchVelocityMoving(
            new Vector2(
                goalDisplacement,
                Field.kGoalHeight
                    - Constants.Camera.kMountingHeight
                    - Constants.Camera.kShooterOffset.y),
            robotVelocity);

    if (launchVelocityData == null) {
      shooter.stop();
      return;
    }

    double launchVelocity = launchVelocityData.x; // meters per second
    double horizontalLaunchAngle = launchVelocityData.y; // degrees clockwise

    double launchRpm = ShooterSubsystem.ballVelocityToFlywheelRpm(launchVelocity);

    turret.trackTarget(true, visionTargetXOffset + horizontalLaunchAngle);

    if (idling) {
      shooter.stop();
    } else {
      // shooterSubsystem.shootRpm(launchRpm);
      SmartDashboard.putNumber("moving shots rpm", launchRpm);
    }
  }

  private void executeSpit() {
    turret.spit(limelight.hasVisibleTarget(), limelight.angleToTarget());
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
    //   executeShoot(true);
    //   towerSubsystem.intake();
    // }
  }

  @Override
  public void end(boolean interrupted) {
    tower.stop();
    shooter.stop();
    turret.idle();
    limelight.enableDrivingMode();
  }
}
