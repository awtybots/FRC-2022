package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootPercent extends StartEndCommand {

  public ShootPercent(double percent, ShooterSubsystem shooterSubsystem) {
    super(
        () -> {
          shooterSubsystem.shootPercent(percent);
        },
        shooterSubsystem::stop,
        shooterSubsystem);
  }
}
