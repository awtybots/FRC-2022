package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class IdleShooter extends CommandBase {

  public static final double kIdleRpm = 500.0;

  private final ShooterSubsystem shooterSubsystem;

  public IdleShooter(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    shooterSubsystem.shootRpm(kIdleRpm);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }
  
}
