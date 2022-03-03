package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.TurnTurretTo;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * spit out bad ball
 *
 * <p>this is a backup command because MovingShots should take care of it
 */
public class Spit extends SequentialCommandGroup {

  public static final double kRpm = 500.0;
  public static final double kDuration = 3.0;

  public Spit(
      ShooterSubsystem shooterSubsystem,
      TowerSubsystem towerSubsystem,
      TurretSubsystem turretSubsystem) {
    addCommands(
        new TurnTurretTo(180.0, turretSubsystem),
        new ShootRpm(kRpm, towerSubsystem, shooterSubsystem).withTimeout(kDuration),
        new TurnTurretTo(0.0, turretSubsystem));
  }
}
