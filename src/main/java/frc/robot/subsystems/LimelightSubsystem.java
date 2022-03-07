package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Field;
import frc.robot.Constants.Limelight;
import frc.robot.util.math.Vector2;
import frc.robot.util.vision.VisionTarget;

public class LimelightSubsystem extends SubsystemBase {

  private final frc.robot.util.vision.Limelight limelight;
  private final VisionTarget upperHub;

  private int xAccumulatorLength = 5;
  private ArrayList<Double> xAccumulator = new ArrayList<>(xAccumulatorLength);

  public LimelightSubsystem() {
    limelight = new RotatableLimelight(Limelight.kMountingHeight, Limelight.kMountingAngle);
    upperHub =
        new VisionTarget(
            limelight, Field.kVisionTargetHeight, Field.kGoalHeight, Limelight.kShooterOffset);

    drivingMode();
  }

  @Override
  public void periodic() {
    if(xAccumulator.size() == xAccumulatorLength) {
      xAccumulator.remove(0);
    }
    if(hasVisibleTarget()) {
      xAccumulator.add(getTargetUnaveragedXOffset());
      getGoalDisplacement();
    }
  }

  /** NOTE: can be null */
  public Vector2 getGoalDisplacement() {
    Vector2 disp = upperHub.getGoalDisplacement().plus(new Vector2(Field.kGoalRadius, 0.0));
    SmartDashboard.putNumber("LL - distance", disp.x);
    return disp;
  }

  public boolean hasVisibleTarget() {
    boolean hasTarget = limelight.hasVisibleTarget();
    SmartDashboard.putBoolean("LL - target", hasTarget);
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
    if(xAccumulator.size() > 0) {
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
