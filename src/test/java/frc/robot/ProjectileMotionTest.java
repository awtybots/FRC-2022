package frc.robot;

import frc.robot.Constants.Field;
import frc.robot.Constants.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.math.ProjectileMotionSolver;
import frc.robot.util.math.ProjectileMotionSolver.CommonProjectiles.Sphere;
import frc.robot.util.math.Vector2;

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

  private static void test() {
    ProjectileMotionSolver projectileMotionSolver =
        new ProjectileMotionSolver(
            Field.kBallMass,
            Sphere.frontalArea(Field.kBallRadius),
            Sphere.dragCoefficient,
            ShooterSubsystem.kMaxBallVelocity,
            ShooterSubsystem.kLaunchAngle);
    projectileMotionSolver.setSimulationStep(0.01);
    projectileMotionSolver.setSimulationIterations(10);

    Vector2 goalDisplacement =
        new Vector2(
            3.0,
            Field.kVisionTargetHeight - (Limelight.kMountingHeight + Limelight.kShooterOffset.y));

    double launchVelStationary =
    projectileMotionSolver.getOptimalLaunchVelocityStationary(goalDisplacement);
    double launchRpmStationary = ShooterSubsystem.ballVelocityToFlywheelRpm(launchVelStationary);

    System.out.println();
    System.out.println("disp: " + goalDisplacement.toString());
    System.out.println("vel: " + launchVelStationary);
    System.out.println("rpm: " + launchRpmStationary);

    for (double j = -5.0; j <= 5.0; j += 1.0) {
      for (double i = -5.0; i <= 5.0; i += 1.0) {
        Vector2 robotVelocity = new Vector2(i, j);

        System.out.println();
        System.out.println("rVel " + robotVelocity.toString());

        Vector2 launchData =
            projectileMotionSolver.getOptimalLaunchVelocityMoving(goalDisplacement, robotVelocity);
        if (launchData == null) {
          System.out.println("no solution :(");
        } else {
          double launchVel = launchData.x;
          double launchRpm = ShooterSubsystem.ballVelocityToFlywheelRpm(launchVel);
          double turretAngle = launchData.y;

          System.out.println("vel: " + launchVel);
          System.out.println("rpm: " + launchRpm);
          System.out.println("theta: " + turretAngle);
        }
      }
    }
  }
}
