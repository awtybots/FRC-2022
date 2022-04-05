package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAim extends CommandBase {
  private final TurretSubsystem turret;
  private final LimelightSubsystem limelight;

  public AutoAim(TurretSubsystem turret) {
    this.turret = turret;
    this.limelight = RobotContainer.Limelight;

    addRequirements(turret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    limelight.enableShootingMode(); // enable every frame in case other command disabled
    turret.trackTarget(limelight.hasVisibleTarget(), limelight.angleToTarget());
  }

  @Override
  public void end(boolean interrupted) {
    turret.idle();
    limelight.enableDrivingMode();
  }
}
