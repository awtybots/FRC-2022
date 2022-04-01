package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerV2Subsystem;

public class ShootRpm extends CommandBase {

  private final TowerV2Subsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final double rpm;

  public ShootRpm(double rpm, TowerV2Subsystem towerSubsystem, ShooterSubsystem shooterSubsystem) {
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.rpm = rpm;

    addRequirements(towerSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.shootRpm(this.rpm);
    RobotContainer.ledSubsystem.blink();
    towerSubsystem.load();
  }

  @Override
  public void execute() {
    towerSubsystem.feed(shooterSubsystem.isAtTarget());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    towerSubsystem.stop();
    RobotContainer.ledSubsystem.turnOn();
  }
}
