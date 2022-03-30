package frc.robot.commands.testing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class ShootRpmSD extends CommandBase {

  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public ShootRpmSD(TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem) {
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(towerSubsystem, shooterSubsystem);

    SmartDashboard.putNumber("SH - set rpm", 0.0);
  }

  @Override
  public void initialize() {
    shooterSubsystem.shootRpm(SmartDashboard.getNumber("SH - set rpm", 0.0));
    towerSubsystem.claim();
    towerSubsystem.load();
  }

  @Override
  public void execute() {
    towerSubsystem.feed(shooterSubsystem.isAtTarget());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    towerSubsystem.free();
    towerSubsystem.stop();
  }
}
