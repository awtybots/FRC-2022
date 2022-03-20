package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootPercent extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;
  private final double speed;

  public ShootPercent(double speed, ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;

    this.speed = speed;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.shootPercent(speed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }
}
