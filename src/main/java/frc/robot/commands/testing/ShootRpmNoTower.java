package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootRpmNoTower extends StartEndCommand {

  public ShootRpmNoTower(double rpm, ShooterSubsystem shooterSubsystem) {
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
