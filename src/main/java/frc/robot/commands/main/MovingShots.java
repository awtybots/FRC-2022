package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Field;
import frc.robot.RobotContainer;
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
  private final DrivetrainSubsystem drivetrain;

  private final ProjectileMotionSolver projectileMotionSolver;

  public MovingShots(
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      DrivetrainSubsystem drivetrainSubsystem) {
    this.tower = towerSubsystem;
    this.shooter = shooterSubsystem;
    this.turret = turretSubsystem;
    this.drivetrain = drivetrainSubsystem;

    addRequirements(towerSubsystem, shooterSubsystem, turretSubsystem);

    projectileMotionSolver =
        new ProjectileMotionSolver(
            Field.kBallMass,
            Sphere.frontalArea(Field.kBallRadius),
            Sphere.dragCoefficient,
            ShooterSubsystem.kLaunchAngle);
  }

  @Override
  public void initialize() {}

  private void executeShoot() {
    if (!RobotContainer.Limelight.hasVisibleTarget()) {
      turret.trackTarget(false);
      shooter.stop();
      return;
    }

    double goalDisplacement = RobotContainer.Limelight.distToTarget();
    if (goalDisplacement == -1) {
      turret.trackTarget(false);
      shooter.stop();
      return;
    }

    double visionTargetXOffset = RobotContainer.Limelight.angleToTarget();
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

    shooter.shootRpm(launchRpm);
  }

  private void executeSpit() {
    turret.spit(
        RobotContainer.Limelight.hasVisibleTarget(), RobotContainer.Limelight.angleToTarget());
  }

  @Override
  public void execute() {
    if (tower.upperBallPresent()) {
      if (tower.upperBallOurs()) {
        executeShoot();
      } else {
        executeSpit();
      }

      tower.feed(turret.isAtTarget() && shooter.isAtTarget());
    } else {
      executeShoot(); // idle shooter
      tower.intake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    tower.stop();
    shooter.stop();
    turret.idle();
  }
}
