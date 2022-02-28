package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurnTurretTo extends CommandBase {

  private final TurretSubsystem turretSubsystem;

  private final double angle;

  public TurnTurretTo(double angle, TurretSubsystem turretSubsystem) {
    addRequirements(turretSubsystem);
    this.turretSubsystem = turretSubsystem;

    this.angle = angle;
  }

  @Override
  public void initialize() {
    turretSubsystem.turnTo(angle);
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.isAtTarget();
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
  }
}
