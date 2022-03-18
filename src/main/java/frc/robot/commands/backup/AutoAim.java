package frc.robot.commands.backup;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAim extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  // TODO tune turret P value
  private final double kP = 0.03; // units: percent output per degree offset

  public AutoAim(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(turretSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.shootingMode();
  }

  @Override
  public void execute() {
    if (limelightSubsystem.hasVisibleTarget()) {
      // simple P controller to track the goal
      double deltaAngle = limelightSubsystem.cameraTargetAngleDelta();
      double motorOutput = -deltaAngle * kP;
      turretSubsystem.drive(motorOutput);
      SmartDashboard.putNumber("AutoAim Turret Output", MathUtil.clamp(motorOutput, -.5, 0.5));
    } else {
      turretSubsystem.seek();
      return;
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
    limelightSubsystem.drivingMode();
  }
}
