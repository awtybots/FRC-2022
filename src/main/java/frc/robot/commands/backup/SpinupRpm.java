package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinupRpm extends StartEndCommand {

  public SpinupRpm(double rpm, ShooterSubsystem shooter) {
    super(() -> shooter.shootRpm(rpm), shooter::stop, shooter);
  }
}
