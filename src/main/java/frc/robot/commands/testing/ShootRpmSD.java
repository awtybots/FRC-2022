package frc.robot.commands.testing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensorsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class ShootRpmSD extends CommandBase {

  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ColorSensorsSubsystem colorSensorsSubsystem;

  public ShootRpmSD(TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem, ColorSensorsSubsystem colorSensorsSubsystem) {
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.colorSensorsSubsystem = colorSensorsSubsystem;

    addRequirements(towerSubsystem, shooterSubsystem);

    SmartDashboard.putNumber("SH - set rpm", 0.0);
  }

  @Override
  public void initialize() {
    shooterSubsystem.shootRpm(SmartDashboard.getNumber("SH - set rpm", 0.0));
  }

  @Override
  public void execute() {
    if (shooterSubsystem.isAtTarget()) {
      if(colorSensorsSubsystem.isUpperBallPresent()) {
        towerSubsystem.feedShooter1();
      } else {
        towerSubsystem.feedShooter2();
      }
    } else {
      towerSubsystem.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    towerSubsystem.stop();
  }
}
