package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Field;
import frc.robot.Constants.Limelight;
import frc.robot.util.math.Vector2;
import frc.robot.util.vision.VisionTarget;
import java.util.ArrayList;

public class LimelightSubsystem extends SubsystemBase {

  private final frc.robot.util.vision.Limelight limelight;
  private final VisionTarget upperHub;

  private int xAccumulatorLength = 10;
  private ArrayList<Double> xAccumulator = new ArrayList<>(xAccumulatorLength);

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
    if (xAccumulator.size() == xAccumulatorLength) {
      xAccumulator.remove(0);
    }
    if (hasVisibleTargetRaw()) {
      xAccumulator.add(getTargetUnaveragedXOffset());
      getTargetXOffset();
      getGoalDisplacement();
    } else if (xAccumulator.size() > 0) {
      xAccumulator.remove(0);
    }
  }

  /** NOTE: can be null */
  public Vector2 getGoalDisplacement() {
    Vector2 disp = upperHub.getGoalDisplacement();
    if (disp == null) {
      SmartDashboard.putNumber("LL - distance", -1.0);
      return null;
    }
    Vector2 dispAdjusted = disp.plus(new Vector2(Field.kGoalRadius, 0.0));
    SmartDashboard.putNumber("LL - distance", dispAdjusted.x);
    return dispAdjusted;
  }

  public boolean hasVisibleTarget() {
    // boolean hasTarget = xAccumulator.size() > 0;
    return hasTargetDebounced;
  }

  public boolean hasVisibleTargetRaw() {
    boolean hasTarget = limelight.hasVisibleTarget();
    hasTargetDebounced = debouncer.calculate(hasTarget);
    SmartDashboard.putBoolean("LL - target", hasTargetDebounced);
    return hasTarget;
  }

  /** degrees NOTE: check for target existing first */
  private double getTargetUnaveragedXOffset() {
    double x = limelight.targetXOffset();
    return x;
  }

  /** degrees NOTE: check for target existing first */
  public double getTargetXOffset() {
    double x = 0.0;
    if (xAccumulator.size() > 0) {
      for (Double xi : xAccumulator) {
        x += xi;
      }
      x /= xAccumulator.size();
    }
    SmartDashboard.putNumber("LL - x", x);
    return x;
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
