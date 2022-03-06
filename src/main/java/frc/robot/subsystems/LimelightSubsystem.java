package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Field;
import frc.robot.Constants.Limelight;
import frc.robot.util.math.Vector2;
import frc.robot.util.vision.VisionTarget;

public class LimelightSubsystem extends SubsystemBase {

  private final frc.robot.util.vision.Limelight limelight;
  private final VisionTarget upperHub;

  public LimelightSubsystem() {
    limelight = new RotatableLimelight(Limelight.kMountingHeight, Limelight.kMountingAngle);
    upperHub =
        new VisionTarget(
            limelight, Field.kVisionTargetHeight, Field.kGoalHeight, Limelight.kShooterOffset);

    drivingMode();
  }

  @Override
  public void periodic() {
    // Vector2 goalDisplacement = getGoalDisplacement();
    // double distance = -1.0;

    // if (goalDisplacement != null) {
    //   distance = goalDisplacement.x;
    // }

    // SmartDashboard.putNumber("LL - distance", distance);
  }

  /** NOTE: can be null */
  public Vector2 getGoalDisplacement() {
    SmartDashboard.putNumber("LL - distance", upperHub.getGoalDisplacement().x);
    return upperHub.getGoalDisplacement();
  }

  public boolean hasVisibleTarget() {
    return limelight.hasVisibleTarget();
  }

  /** degrees NOTE: check for target existing first */
  public double getTargetXOffset() {
    return limelight.targetXOffset();
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
