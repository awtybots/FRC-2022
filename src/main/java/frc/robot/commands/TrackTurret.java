package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TrackTurret extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  public TrackTurret(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
  }

  @Override
  public void initialize() {
    limelightSubsystem.shootingMode();
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
    limelightSubsystem.drivingMode();
  }
}
