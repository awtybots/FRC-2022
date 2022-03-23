package frc.robot.commands.main;

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
    turretSubsystem.trackTarget(limelightSubsystem.hasVisibleTarget(), limelightSubsystem.cameraTargetAngleDelta());
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.idle();
    limelightSubsystem.drivingMode();
  }
}
