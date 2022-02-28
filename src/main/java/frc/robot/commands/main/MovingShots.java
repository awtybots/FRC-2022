package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Field;
import frc.robot.subsystems.ColorSensorsSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.math.ProjectileMotionSolver;
import frc.robot.util.math.ProjectileMotionSolver.CommonProjectiles.Sphere;

public class MovingShots extends CommandBase {
  private final TowerSubsystem towerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final ColorSensorsSubsystem colorSensorsSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  private final ProjectileMotionSolver projectileMotionSolver;

  public MovingShots(
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

    addRequirements(
        towerSubsystem,
        shooterSubsystem,
        turretSubsystem,
        limelightSubsystem); // color sensor subsystem not a requirement

    projectileMotionSolver =
        new ProjectileMotionSolver(
            Field.kBallMass, Sphere.frontalArea(Field.kBallRadius), Sphere.dragCoefficient);
  }

  @Override
  public void initialize() {
    limelightSubsystem.shootingMode();
  }

  @Override
  public void execute() {

    // TODO write autoshooting code
  }

  @Override
  public void end(boolean interrupted) {
    towerSubsystem.stop();
    shooterSubsystem.stop();
    turretSubsystem.stop();
    limelightSubsystem.drivingMode();
  }
}
