package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensorsSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootRpmOrSpit extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;
  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final ColorSensorsSubsystem colorSensorsSubsystem;

  private final double rpm;

  public ShootRpmOrSpit(
      double rpm,
      IntakeSubsystem intakeSubsystem,
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      LimelightSubsystem limelightSubsystem,
      ColorSensorsSubsystem colorSensorsSubsystem) {
    
    this.intakeSubsystem = intakeSubsystem;
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.colorSensorsSubsystem = colorSensorsSubsystem;

    this.rpm = rpm;

    addRequirements(intakeSubsystem, towerSubsystem, shooterSubsystem, turretSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.shootingMode();
    intakeSubsystem.start();
  }

  private void executeShoot() {
    shooterSubsystem.shootRpm(this.rpm);

    if (!limelightSubsystem.hasVisibleTarget()) {
      turretSubsystem.seek();
      return;
    }

    double deltaAngle = limelightSubsystem.getTargetXOffset();

    turretSubsystem.turnBy(deltaAngle);
  }

  private void executeSpit() {
    shooterSubsystem.spit();
    turretSubsystem.spit();
  }

  @Override
  public void execute() {
    if (colorSensorsSubsystem.isUpperBallPresent()) {
      if (colorSensorsSubsystem.isUpperBallOurs()) {
        executeShoot();
      } else {
        executeSpit();
      }

      if (turretSubsystem.isAtTarget() && shooterSubsystem.isAtTarget()) {
        towerSubsystem.feedShooter();
      } else {
        towerSubsystem.stop();
      }
    } else {
      executeShoot();
      towerSubsystem.intake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    shooterSubsystem.stop();
    towerSubsystem.stop();
    turretSubsystem.stop();
    limelightSubsystem.drivingMode();
  }
}
