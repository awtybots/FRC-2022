package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.util.math.Interpolatable;
import frc.robot.util.math.InterpolationMap;
import frc.robot.util.math.Vector2;

public class ShootInterpolated extends CommandBase {
  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  public static final InterpolationMap<Double> iMap = new InterpolationMap<>();

  static {
    iMap.addKeyframe(3.0, Interpolatable.interpolatableDouble(1600));
    iMap.addKeyframe(3.5, Interpolatable.interpolatableDouble(1600));
    iMap.addKeyframe(4.0, Interpolatable.interpolatableDouble(1700));
    iMap.addKeyframe(4.5, Interpolatable.interpolatableDouble(1800));
    iMap.addKeyframe(6.0, Interpolatable.interpolatableDouble(2000));
  }

  public ShootInterpolated(
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem) {
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(towerSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.shootingMode();
  }

  @Override
  public void execute() {
    if (!limelightSubsystem.hasVisibleTarget()) {
      shooterSubsystem.stop();
      return;
    }

    Vector2 goalDisplacement = limelightSubsystem.getGoalDisplacement();
    if (goalDisplacement == null) {
      shooterSubsystem.stop();
      return;
    }

    double launchRpm = iMap.get(goalDisplacement.x);
    shooterSubsystem.shootRpm(launchRpm);

    if (shooterSubsystem.isAtTarget()) {
      towerSubsystem.feedShooter();
    } else {
      towerSubsystem.stopUpper();
    }
  }

  @Override
  public void end(boolean interrupted) {
    towerSubsystem.stop();
    shooterSubsystem.stop();
    limelightSubsystem.drivingMode();
  }
}
