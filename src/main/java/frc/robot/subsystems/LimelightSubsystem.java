package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Field;
import frc.robot.Constants.Limelight;
import frc.robot.util.vision.VisionTarget;

public class LimelightSubsystem extends SubsystemBase {

  private final frc.robot.util.vision.Limelight limelight;
  private final VisionTarget upperHub;

  private MedianFilter distFilter = new MedianFilter(5);
  private double distToTarget = 0;
  private MedianFilter xFilter = new MedianFilter(5);
  private double cameraAngleDelta = 0;

  private boolean hasTargetDebounced = false;
  private Debouncer debouncer = new Debouncer(0.5, DebounceType.kFalling);

  public LimelightSubsystem() {
    limelight = new RotatableLimelight(Limelight.kMountingHeight, Limelight.kMountingAngle);
    upperHub =
        new VisionTarget(
            limelight, Field.kVisionTargetHeight, Field.kGoalHeight, Limelight.kShooterOffset);

    drivingMode();
  }

  @Override
  public void periodic() {
    if (hasVisibleTarget()) {
      cameraAngleDelta = xFilter.calculate(limelight.targetXOffset());
      distToTarget = distFilter.calculate(upperHub.getGoalDisplacement().x + Field.kGoalRadius);
    } else {
      cameraAngleDelta = 0;
      distToTarget = -1;
    }
  }

  public boolean hasVisibleTarget() {
    boolean hasTarget = limelight.hasVisibleTarget();
    hasTargetDebounced = debouncer.calculate(hasTarget);
    return hasTargetDebounced;
  }

  public double distToTarget() {
    return distToTarget;
  }

  public double cameraTargetAngleDelta() {
    return cameraAngleDelta;
  }

  public void drivingMode() {
    limelight.setPipeline(Limelight.kPipelineDriving);
  }

  public void shootingMode() {
    limelight.setPipeline(Limelight.kPipelineShooting);
  }

  private class RotatableLimelight extends frc.robot.util.vision.Limelight {

    public RotatableLimelight(double mountingHeight, double mountingAngle) {
      super(mountingHeight, mountingAngle);
    }

    @Override
    public double targetXOffset() {
      switch (Limelight.kMountingDirection) {
        case kLandscape:
          return -super.targetXOffset();
        case kPortrait:
          return super.targetYOffset();
      }
      return 0.0;
    }

    @Override
    public double targetYOffset() {
      switch (Limelight.kMountingDirection) {
        case kLandscape:
          return -super.targetYOffset();
        case kPortrait:
          return -super.targetXOffset();
      }
      return 0.0;
    }
  }
}
