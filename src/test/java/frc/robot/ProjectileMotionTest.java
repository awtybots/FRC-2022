package frc.robot;

import frc.robot.Constants.Field;
import frc.robot.Constants.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.math.Vector2;
import frc.robot.util.math.ProjectileMotionSolver.CommonProjectiles.Sphere;

public class ProjectileMotionTest {
  public static void main(String[] args) {
    // for (double j = -1.0; j <= 1.0; j += 2.0) {
    //     for (double i = -1.0; i <= 1.0; i += 1.0) {
    //     Vector2 robotVelocity = new Vector2(i, j); // moving x m/s right
    //     test(robotVelocity);
    //   }
    // }
    test();
  }

  // private static void test(Vector2 robotVelocity) {
  //   ProjectileMotionSolver projectileMotionSolver =
  //       new ProjectileMotionSolver(
  //           Field.kBallMass,
  //           Sphere.frontalArea(Field.kBallRadius),
  //           Sphere.dragCoefficient,
  //           ShooterSubsystem.kLaunchAngle);
    
  //   Vector2 goalDisplacement = new Vector2(3.0, Field.kVisionTargetHeight - (Limelight.kMountingHeight + Limelight.kShooterOffset.y));
    
  //   double v_j = -robotVelocity.y;
  //   double cos_b = Math.cos(Math.toRadians(ShooterSubsystem.kLaunchAngle));
  //   double sin_b = Math.sin(Math.toRadians(ShooterSubsystem.kLaunchAngle));

  //   double v_min = 0.0;
  //   double v_max = 3.0;//ShooterSubsystem.kMaxBallVelocity;
  //   double v = (v_min + v_max) / 2.0;

  //   double v_ij = v * cos_b;
  //   double theta = Math.toDegrees(Math.asin(v_j / v_ij));

  //   System.out.println();
  //   System.out.println("R_i: " + robotVelocity.x);
  //   System.out.println("R_j: " + robotVelocity.y);
  //   System.out.println("v_ij: " + v_ij);
  //   System.out.println("theta: " + theta);

  //   // projectileMotionSolver.getOptimalLaunchVelocityStationary(goalPosition);
  // }

  private static void test() {
    ProjectileMotionTestSolver projectileMotionSolver =
        new ProjectileMotionTestSolver(
            Field.kBallMass,
            Sphere.frontalArea(Field.kBallRadius),
            Sphere.dragCoefficient,
            ShooterSubsystem.kLaunchAngle);
    projectileMotionSolver.setSimulationStep(0.01);
    
    Vector2 goalDisplacement = new Vector2(1.0, Field.kVisionTargetHeight - (Limelight.kMountingHeight + Limelight.kShooterOffset.y));
    Vector2 robotVelocity = new Vector2(0.5, 0.5);

    Vector2 launchData = projectileMotionSolver.getOptimalLaunchVelocityMoving(goalDisplacement, robotVelocity);
    if(launchData != null) {
      double launchVel = launchData.x;
      double turretAngle = launchData.y;
      System.out.println();
      System.out.println("vel: " + launchVel);
      System.out.println("theta: " + turretAngle);
    }

  }
}
