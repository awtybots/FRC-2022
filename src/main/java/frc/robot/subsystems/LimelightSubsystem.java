package frc.robot.subsystems;

import frc.robot.Constants.Limelight;
import frc.robot.Constants.Field;
import frc.robot.util.math.Vector2;
import frc.robot.util.vision.VisionTarget;

public class LimelightSubsystem extends frc.robot.util.vision.Limelight {

  private final VisionTarget visionTarget;

  public LimelightSubsystem() {
    super(Limelight.mountingHeight, Limelight.mountingAngle);

    visionTarget = new VisionTarget(this, Field.visionTargetHeight, Field.goalHeight);
  }

  @Override
  public double targetXOffset() {
    switch(Limelight.mountingDirection) {
      case kLandscape:
        return -super.targetXOffset();
      case kPortrait:
        return super.targetYOffset();
    }
    return 0.0;
  }

  @Override
  public double targetYOffset() {
    switch(Limelight.mountingDirection) {
      case kLandscape:
        return -super.targetYOffset();
      case kPortrait:
        return super.targetXOffset();
    }
    return 0.0;
  }

  public Vector2 getGoalDisplacement() {
    return visionTarget.getGoalDisplacement();
  }
  
}
