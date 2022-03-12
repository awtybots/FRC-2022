package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.util.math.Convert;
import frc.robot.util.math.Interpolatable;
import frc.robot.util.math.InterpolationMap;
import frc.robot.util.math.Vector2;

public class ShootInterpolated extends CommandBase {
  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final ColorSensorsSubsystem colorSensorsSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  private boolean alreadySet = false;
  
  private final InterpolationMap<Double> iMap = new InterpolationMap<>();

  public ShootInterpolated(
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      ColorSensorsSubsystem colorSensorsSubsystem,
      LimelightSubsystem limelightSubsystem) {
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.colorSensorsSubsystem = colorSensorsSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    initIMap();
    
    addRequirements(
        towerSubsystem,
        shooterSubsystem,
        turretSubsystem,
        limelightSubsystem); // drive and color sensor subsystems not requirements
  }

  private void initIMap() {
    iMap.addKeyframe(Convert.feetToMeters(9, 0), Interpolatable.interpolatableDouble(1500));
    iMap.addKeyframe(Convert.feetToMeters(12, 0), Interpolatable.interpolatableDouble(1700));
    iMap.addKeyframe(Convert.feetToMeters(16, 0), Interpolatable.interpolatableDouble(2300));
  }

  @Override
  public void initialize() {
    limelightSubsystem.shootingMode();
  }

  private void executeShoot(boolean idling) {
    alreadySet = false;

    if (!limelightSubsystem.hasVisibleTarget()) {
      turretSubsystem.seek();
      shooterSubsystem.stop();
      return;
    }

    Vector2 goalDisplacement = limelightSubsystem.getGoalDisplacement();
    if (goalDisplacement == null) {
      turretSubsystem.seek();
      shooterSubsystem.stop();
      return;
    }

    double launchRpm = iMap.get(goalDisplacement.x);
    if (idling) {
      shooterSubsystem.stop();
    } else {
      shooterSubsystem.shootRpm(launchRpm);
    }

    double visionTargetXOffset = limelightSubsystem.getTargetXOffset();
    turretSubsystem.turnBy(visionTargetXOffset);
  }

  private void executeSpit() {
    if (!alreadySet) {
      alreadySet = true;
      if (limelightSubsystem.hasVisibleTarget()) {
        turretSubsystem.spitRelative(limelightSubsystem.getTargetXOffset());
      } else {
        turretSubsystem.spit();
      }
    }
    shooterSubsystem.spit();
  }

  @Override
  public void execute() {
    if (colorSensorsSubsystem.isUpperBallPresent()) {
      if (colorSensorsSubsystem.isUpperBallOurs()) {
        executeShoot(false);
      } else {
        executeSpit();
      }

      if (turretSubsystem.isAtTarget() && shooterSubsystem.isAtTarget()) {
        towerSubsystem.feedShooter();
      } else {
        towerSubsystem.stopUpper();

        if (colorSensorsSubsystem.isLowerBallPresent()) {
          towerSubsystem.stop();
        }
      }
    } else {
      executeShoot(true);
      towerSubsystem.intake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    towerSubsystem.stop();
    shooterSubsystem.stop();
    turretSubsystem.stop();
    limelightSubsystem.drivingMode();
  }
  
}
