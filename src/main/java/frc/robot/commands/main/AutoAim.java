package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAim extends CommandBase {
  private final TurretSubsystem turret;
  private final LimelightSubsystem limelight;

  public AutoAim(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
    this.turret = turretSubsystem;
    this.limelight = limelightSubsystem;

    addRequirements(turretSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelight.enableShootingMode();
  }

  @Override
  public void execute() {
    turret.trackTarget(limelight.hasVisibleTarget(), limelight.angleToTarget());
  }

  @Override
  public void end(boolean interrupted) {
    turret.idle();
    limelight.enableDrivingMode();
  }
}
