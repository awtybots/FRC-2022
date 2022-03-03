package frc.robot;

import frc.robot.Constants.Field;
import frc.robot.Constants.Shooter;
import frc.robot.util.math.ProjectileMotionSolver;
import frc.robot.util.math.ProjectileMotionSolver.CommonProjectiles.Sphere;

public class ProjectileMotionTest {
  public static void main(String[] args) {
    ProjectileMotionSolver projectileMotionSolver =
        new ProjectileMotionSolver(
            Field.kBallMass,
            Sphere.frontalArea(Field.kBallRadius),
            Sphere.dragCoefficient,
            Shooter.kLaunchAngle);
    
    // projectileMotionSolver.getOptimalLaunchVelocityStationary(goalPosition);
  }
}
