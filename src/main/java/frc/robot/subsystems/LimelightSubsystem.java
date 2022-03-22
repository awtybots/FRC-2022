package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Field;
import frc.robot.Constants.Limelight;
import frc.util.math.Vector2;
import frc.util.vision.VisionTarget;

public class LimelightSubsystem extends SubsystemBase {

  private final frc.util.vision.Limelight limelight;
  private final VisionTarget upperHub;

  private final int filterSize = 10;

  private MedianFilter distFilter = new MedianFilter(filterSize);
  private double distToTarget = 0;
  private MedianFilter xFilter = new MedianFilter(filterSize);
  private double cameraAngleDelta = 0;

  private boolean hasTargetDebounced = false;
  private Debouncer debouncer = new Debouncer(filterSize * 0.02, DebounceType.kFalling);

  public LimelightSubsystem() {
    limelight = new RotatableLimelight(Limelight.kMountingHeight, Limelight.kMountingAngle);
    upperHub =
        new VisionTarget(
            limelight, Field.kVisionTargetHeight, Field.kGoalHeight, Limelight.kShooterOffset);

    drivingMode();
  }

  @Override
  public void periodic() {
    boolean hasTarget = hasVisibleTarget();
    Vector2 targetVector = upperHub.getGoalDisplacement();
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
    limelight.setPipeline(Limelight.kPipelineDriving);
  }

  public void shootingMode() {
    limelight.setPipeline(Limelight.kPipelineShooting);
  }

  private class RotatableLimelight extends frc.util.vision.Limelight {

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
