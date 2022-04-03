package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootPercent extends StartEndCommand {

  public ShootPercent(double percent, ShooterSubsystem shooter) {
    super(() -> shooter.shootPercent(percent), shooter::stop, shooter);
  }
}
