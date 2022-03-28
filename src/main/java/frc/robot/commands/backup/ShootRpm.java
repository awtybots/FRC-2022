package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ColorSensorsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class ShootRpm extends CommandBase {

  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ColorSensorsSubsystem colorSensorsSubsystem;
  private final double rpm;

  public ShootRpm(
      double rpm,
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      ColorSensorsSubsystem colorSensorsSubsystem) {
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.colorSensorsSubsystem = colorSensorsSubsystem;
    this.rpm = rpm;

    addRequirements(towerSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.shootRpm(this.rpm);
    RobotContainer.ledSubsystem.blink();
  }

  @Override
  public void execute() {
    final boolean readyToShoot = shooterSubsystem.isAtTarget();
    final boolean upperBallPresent = colorSensorsSubsystem.upperBallPresent();

    if (!readyToShoot) towerSubsystem.stop();

    if (readyToShoot && upperBallPresent) towerSubsystem.feedFromUpper();
    if (readyToShoot && !upperBallPresent) towerSubsystem.feedFromLower();
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    towerSubsystem.stop();
    RobotContainer.ledSubsystem.turnOn();
  }
}
