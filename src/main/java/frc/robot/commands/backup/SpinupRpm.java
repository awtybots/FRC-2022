package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinupRpm extends CommandBase {

  private final ShooterSubsystem shooter;
  private final double rpm;

  public SpinupRpm(double rpm, ShooterSubsystem shooterSubsystem) {
    this.shooter = shooterSubsystem;
    this.rpm = rpm;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooter.shootRpm(this.rpm);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
}
