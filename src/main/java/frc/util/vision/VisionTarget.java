package frc.util.vision;

import frc.util.math.Vector2;

public class VisionTarget {
  private final Limelight limelight;

  private final double visionTargetY;
  private final double goalY;
  private final Vector2 mechanismOffset;

  public VisionTarget(
      Limelight limelight, double visionTargetHeight, double goalHeight, Vector2 mechanismOffset) {
    this.limelight = limelight;

    this.visionTargetY = visionTargetHeight;
    this.goalY = goalHeight;
    this.mechanismOffset = mechanismOffset;
  }

  /**
   * Get the relative displacement vector from the mechanism to the goal. The x axis is horizontal
   * displacement and the y axis is vertical displacement. Values are in meters.
   *
   * @return The displacement vector, or null if no vision target is detected.
   */
  public Vector2 getGoalDisplacement() {
    if (!limelight.hasVisibleTarget()) return null;

    double angleY = limelight.targetYOffset();

    double opposite = visionTargetY - limelight.getMountingHeight();
    double tangent = Math.tan(Math.toRadians(limelight.getMountingAngle() + angleY));
    double adjacent = opposite / tangent;

    return new Vector2(adjacent, goalY - limelight.getMountingHeight()).minus(mechanismOffset);
  }
}
