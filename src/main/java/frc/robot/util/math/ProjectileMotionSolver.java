package frc.robot.util.math;

public class ProjectileMotionSolver {
  private static final double g = 9.81; // acceleration due to gravity (m/s^2)
  private static final double rho = 1.225; // density of air (kg/m^3)

  public static class CommonProjectiles {
    public static class Sphere {
      public static final double dragCoefficient = 0.47;

      public static double frontalArea(double radius) {
        return radius * radius * Math.PI;
      }
    }
  }

  private final double projectileMass;
  private final Vector2 projectileWeight;
  private final double projectileDragFactor;

  private double simulationStep = 0.01;
  private double simulationIterations = 10;

  private double launchAngle;
  private double maxLaunchSpeed;

  public ProjectileMotionSolver(
      double projectileMass,
      double projectileFrontalArea,
      double projectileDragCoefficient,
      double maxLaunchSpeed) {
    this.projectileMass = projectileMass;
    this.projectileWeight = new Vector2(0, projectileMass * -g);
    this.projectileDragFactor = 0.5 * rho * projectileDragCoefficient * projectileFrontalArea;
    this.maxLaunchSpeed = maxLaunchSpeed;
  }

  public ProjectileMotionSolver(
      double projectileMass,
      double projectileFrontalArea,
      double projectileDragCoefficient,
      double maxLaunchSpeed,
      double launchAngle) {
    this(projectileMass, projectileFrontalArea, projectileDragCoefficient, maxLaunchSpeed);
    setLaunchAngle(launchAngle);
  }

  /**
   * @param launchAngle The angle from the horizontal, in degrees, that the projectile begins motion
   *     in. For reference, 90 is completely vertical and 0 is completely horizontal.
   */
  public void setLaunchAngle(double launchAngle) {
    this.launchAngle = launchAngle;
  }

  /**
   * @param simulationStep Number of seconds between each "tick" in the simulation, 0.01 by default.
   */
  public void setSimulationStep(double simulationStep) {
    this.simulationStep = simulationStep;
  }

  /**
   * @param simulationIterations Number of iterations of the simulation completed to find the
   *     optimal launch velocity, 10 by default. Each iteration doubles the precision using a binary
   *     search.
   */
  public void setSimulationIterations(double simulationIterations) {
    this.simulationIterations = simulationIterations;
  }

  private Vector2 getDragForce(Vector2 velocity) {
    double v = velocity.getMagnitude();
    double dragForceMagnitude = projectileDragFactor * v * v;
    double dragForceAngle = velocity.getAngle() + 180.0;
    return Vector2.fromPolar(dragForceMagnitude, dragForceAngle);
  }

  /**
   * Get the optimal launch velocity for a stationary shot.
   *
   * @param goalPosition A Vector2 representing the goal's relative position from the launching
   *     position of the projectile, where x is the horizontal distance of the goal and y is the
   *     height of the goal.
   * @return The optimal launch velocity, in meters per second, for the projectile to reach that
   *     position. NOTE: may return NaN.
   */
  public double getOptimalLaunchVelocityStationary(Vector2 goalPosition) {
    Vector2 minLaunchVelocity = new Vector2();
    Vector2 maxLaunchVelocity = Vector2.fromPolar(maxLaunchSpeed, launchAngle);

    double intersectYMax = runSingleSimulation(goalPosition.x, maxLaunchVelocity);
    if (intersectYMax < goalPosition.y) return Double.NaN;

    // binary search for goldilocks velocity
    // uncertainty on this velocity is halved with each iteration
    for (int i = 0; i < simulationIterations; i++) {
      Vector2 middleLaunchVelocity = minLaunchVelocity.plus(maxLaunchVelocity).scaled(0.5);
      double intersectY = runSingleSimulation(goalPosition.x, middleLaunchVelocity);
      if (intersectY < goalPosition.y) {
        minLaunchVelocity = middleLaunchVelocity;
      } else {
        maxLaunchVelocity = middleLaunchVelocity;
      }
    }

    return minLaunchVelocity.plus(maxLaunchVelocity).scaled(0.5).getMagnitude();
  }

  /**
   * Get the optimal launch velocity and turret offset angle for a moving shot. All units are in
   * terms of meters, seconds, and degrees.
   *
   * @param goalDisplacement A Vector2 representing the goal's relative position from the launching
   *     position of the projectile, where x is the horizontal distance of the goal and y is the
   *     height of the goal.
   * @param robotVelocity A Vector2 of the robot's velocity, where the x term is in units parallel
   *     to the goal direction, and the y term is in units perpendicular to the goal direction.
   * @return A Vector2 where x is the launch velocity and y is the turret offset angle. NOTE: may be
   *     null
   */
  public Vector2 getOptimalLaunchVelocityMoving(Vector2 goalDisplacement, Vector2 robotVelocity) {
    if (robotVelocity.x == 0.0) robotVelocity.x = 0.0001; // cheese
    if (robotVelocity.y == 0.0) robotVelocity.y = 0.0001;

    double v_j = -robotVelocity.y;
    double v_max = maxLaunchSpeed;
    double cos_b = Math.cos(Math.toRadians(launchAngle));
    double tan_b = Math.tan(Math.toRadians(launchAngle));

    double thetaMin = Math.toDegrees(v_j / v_max / cos_b);
    double thetaMax = Math.toDegrees(Math.atan2(-v_j, robotVelocity.x));

    if (robotVelocity.y < 0.0) { // moving left
      thetaMax += 180.0;
    } else if (robotVelocity.y > 0.0) { // moving right
      thetaMax -= 180.0;
    }

    double aMin = robotVelocity.x - robotVelocity.y / Math.tan(Math.toRadians(thetaMax));
    double aMax = robotVelocity.x - robotVelocity.y / Math.tan(Math.toRadians(thetaMin));

    // System.out.println("theta min: " + thetaMin);
    // System.out.println("theta max: " + thetaMax);
    // System.out.println("a min: " + aMin);
    // System.out.println("a max: " + aMax);
    // System.out.println();

    for (int i = 0; i < simulationIterations; i++) {
      double a = (aMin + aMax) / 2.0;
      double v_i = a - robotVelocity.x;
      double v_ij = Math.sqrt(v_i * v_i + v_j * v_j);
      double v_k = v_ij * tan_b;
      Vector2 launchVelocity = new Vector2(a, v_k);
      double crossHeight = runSingleSimulation(goalDisplacement.x, launchVelocity);

      // System.out.println("launch: " + launchVelocity.toString());
      // System.out.println("theta: " + Math.toDegrees(Math.atan2(v_j, v_i));
      // System.out.println("dy: " + (crossHeight - goalDisplacement.y));
      // System.out.println();

      if (crossHeight > goalDisplacement.y) { // too high, need less v_k
        aMax = a;
      } else { // too low, need more v_k
        aMin = a;
      }

      if (i == simulationIterations - 1) {
        // System.out.println("a: " + a);

        double dy = crossHeight - goalDisplacement.y;
        if (Math.abs(dy) > 0.1) {
          // System.out.println(" ==== HIGH DY: " + dy);
          return null;

        } else {
          double v = Math.sqrt(v_i * v_i + v_j * v_j) / cos_b;
          double theta = Math.toDegrees(Math.atan2(v_j, v_i));

          return new Vector2(v, theta);
        }
      }
    }
    return null;
  }

  private double runSingleSimulation(double goalX, Vector2 launchVelocity) {
    Vector2 position = new Vector2();
    Vector2 lastPosition = new Vector2();
    Vector2 velocity = launchVelocity.clone();

    double time = 0.0;

    // System.out.println("p: " + position);
    // System.out.println("v: " + velocity);

    while (position.x < goalX && velocity.x > 0.001 && time < 5.0) {
      lastPosition = position.clone();
      position = position.plus(velocity.scaled(simulationStep));

      Vector2 netForce = projectileWeight.plus(getDragForce(velocity));
      Vector2 acceleration = netForce.scaled(1.0 / projectileMass);

      velocity = velocity.plus(acceleration.scaled(simulationStep));
      time += simulationStep;

      // System.out.println();
      // System.out.println("t: " + time);
      // System.out.println("d: " + getDragForce(velocity));
      // System.out.println("p: " + position);
      // System.out.println("v: " + velocity);
      // System.out.println("a: " + acceleration);
    }

    if (velocity.y > 0.0) { // velocity must be negative for 2022 game
      return Double.POSITIVE_INFINITY;

    } else if (position.x >= goalX) {
      double alpha = (goalX - lastPosition.x) / (position.x - lastPosition.x);
      double intersectY = (position.y - lastPosition.y) * alpha + lastPosition.y;
      return intersectY;

    } else {
      return Double.NEGATIVE_INFINITY;
    }
  }
}
