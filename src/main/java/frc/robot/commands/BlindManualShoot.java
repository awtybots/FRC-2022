package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class BlindManualShoot extends StartEndCommand {

  public BlindManualShoot(double rpm, ShooterSubsystem shooterSubsystem) {
    super(
        () -> {
          shooterSubsystem.shootRpm(rpm);
        },
        () -> {
          shooterSubsystem.stop();
        },
        shooterSubsystem);
  }
}
