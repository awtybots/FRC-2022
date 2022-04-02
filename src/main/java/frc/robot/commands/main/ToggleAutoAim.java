package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.backup.DriveTurret;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ToggleAutoAim extends InstantCommand {

  public ToggleAutoAim(DriveTurret driveTurretCommand, TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
    super(() -> {
      if(turretSubsystem.getDefaultCommand() instanceof AutoAim) {
        turretSubsystem.setDefaultCommand(driveTurretCommand);
      } else {
        turretSubsystem.setDefaultCommand(new AutoAim(turretSubsystem, limelightSubsystem));
      }
    });
  }

}
