package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualShoot extends StartEndCommand {

  public ManualShoot(double rpm, ShooterSubsystem shooterSubsystem) {
    super(
        () -> {
          shooterSubsystem.setRpm(rpm);
        },
        () -> {
          shooterSubsystem.stop();
        },
        shooterSubsystem);
  }
}
