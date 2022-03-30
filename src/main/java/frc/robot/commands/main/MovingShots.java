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
  private final LimelightSubsystem limelightSubsystem;
  private final DrivetrainSubsystem drivetrainSubsystem;

  private final ProjectileMotionSolver projectileMotionSolver;
  private double launchRpm;

  public MovingShots(
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      DrivetrainSubsystem drivetrainSubsystem,
      LimelightSubsystem limelightSubsystem) {
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(
        towerSubsystem,
        shooterSubsystem,
        turretSubsystem,
        limelightSubsystem); // drive subsystem only observed

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
    towerSubsystem.claim();
  }

  private void aimForShooting() {
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

    launchRpm = ShooterSubsystem.ballVelocityToFlywheelRpm(launchVelocity);

    turretSubsystem.trackTarget(true, visionTargetXOffset + horizontalLaunchAngle);

    SmartDashboard.putNumber("moving shots rpm", launchRpm);
  }

  private void aimForSpitting() {
    turretSubsystem.spit(
        limelightSubsystem.hasVisibleTarget(), limelightSubsystem.cameraTargetAngleDelta());
  }

  private void shoot() {
    shooterSubsystem.shootRpm(launchRpm);
  }

  private void spit() {
    shooterSubsystem.spit();
  }

  @Override
  public void execute() {
    if (towerSubsystem.upperBallOurs()) {
      aimForShooting();
      shoot();
    } else {
      aimForSpitting();
      spit();
    }

    towerSubsystem.feed(turretSubsystem.isAtTarget() && shooterSubsystem.isAtTarget());
  }

  @Override
  public void end(boolean interrupted) {
    towerSubsystem.stop();
    towerSubsystem.free();
    shooterSubsystem.stop();
    turretSubsystem.idle();
    limelightSubsystem.drivingMode();
  }
}
