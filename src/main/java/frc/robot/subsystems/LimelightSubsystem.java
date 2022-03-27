package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Camera;
import frc.robot.Constants.Field;
import frc.util.math.Vector2;
import frc.util.vision.Limelight;
import frc.util.vision.VisionTarget;

public class LimelightSubsystem extends SubsystemBase {

  private final Limelight limelight;
  private final VisionTarget upperHub;

  private final int filterSize = 10;

  private MedianFilter distFilter = new MedianFilter(filterSize);
  private double distToTarget = 0;
  private MedianFilter xFilter = new MedianFilter(filterSize);
  private double cameraAngleDelta = 0;

  private boolean hasTargetDebounced = false;
  private Debouncer debouncer = new Debouncer(filterSize * 0.02, DebounceType.kFalling);

  public LimelightSubsystem() {
    limelight =
        new RotatableLimelight(
            Camera.kMountingHeight, Camera.kMountingAngle, Camera.kShooterOffset);
    upperHub =
        new VisionTarget(Field.kVisionTargetHeight, Field.kGoalHeight, Camera.kShooterOffset);

    drivingMode();
  }

  @Override
  public void periodic() {
    boolean hasTarget = hasVisibleTarget();
    Vector2 targetVector = limelight.getDisplacementFrom(upperHub);

    if (hasTarget && targetVector != null) {
      cameraAngleDelta = xFilter.calculate(limelight.targetXOffset());
      distToTarget = distFilter.calculate(targetVector.x + Field.kGoalRadius);
    } else {
      cameraAngleDelta = 0;
      distToTarget = -1;
    }

    SmartDashboard.putBoolean("Target Visible", hasTarget);
    SmartDashboard.putNumber("Camera Angle Delta", cameraAngleDelta);
    SmartDashboard.putNumber("Distance to Target", distToTarget);
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
    limelight.setPipeline(Camera.kPipelineDriving);
  }

  public void shootingMode() {
    limelight.setPipeline(Camera.kPipelineShooting);
  }

  private class RotatableLimelight extends frc.util.vision.Limelight {

    public RotatableLimelight(
        double mountingHeight, double mountingAngle, Vector2 mechanismOffset) {
      super(mountingHeight, mountingAngle, mechanismOffset);
    }

    @Override
    public double targetXOffset() {
      switch (Camera.kMountingDirection) {
        case kLandscape:
          return -super.targetXOffset();
        case kPortrait:
          return super.targetYOffset();
      }
      return 0.0;
    }

    @Override
    public double targetYOffset() {
      switch (Camera.kMountingDirection) {
        case kLandscape:
          return -super.targetYOffset();
        case kPortrait:
          return -super.targetXOffset();
      }
      return 0.0;
    }
  }
}
