package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooter;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class ShootInterpolated extends CommandBase {
  private final TowerSubsystem tower;
  private final ShooterSubsystem shooter;

  public ShootInterpolated(TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem) {
    this.tower = towerSubsystem;
    this.shooter = shooterSubsystem;

    addRequirements(towerSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!RobotContainer.Limelight.hasVisibleTarget()) {
      shooter.stop();
      return;
    }

    double goalDistance = RobotContainer.Limelight.distToTarget();
    if (goalDistance == -1) {
      shooter.stop();
      return;
    }

    double launchRpm = Shooter.shotMap.calculateShot(goalDistance);
    shooter.shootRpm(launchRpm);

    tower.feed(shooter.isAtTarget());
  }

  @Override
  public void end(boolean interrupted) {
    tower.stop();
    shooter.stop();
  }
}
