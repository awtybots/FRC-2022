package frc.util.vision;

import frc.util.math.Vector2;

public class VisionTarget {

  public final double height;
  public final double goalHeight;
  public final Vector2 cameraToMechanismOffset;

  public VisionTarget(double visionTargetHeight, double goalHeight, Vector2 mechanismOffset) {
    this.height = visionTargetHeight;
    this.goalHeight = goalHeight;
    this.cameraToMechanismOffset = mechanismOffset;
  }
}
