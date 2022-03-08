package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAim extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private final LimelightSubsystem limelightSubsystem;

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
    if (!limelightSubsystem.hasVisibleTarget()) {
      turretSubsystem.seek();
      return;
    }

    double deltaAngle = limelightSubsystem.getTargetXOffset();
    turretSubsystem.turnBy(deltaAngle);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
    limelightSubsystem.drivingMode();
  }
}
