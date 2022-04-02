package frc.robot.auton.smart;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auton.trajectories.FourBall0;
import frc.robot.auton.trajectories.FourBall1;
import frc.robot.commands.backup.ShootRpm;
import frc.robot.commands.main.AutoAim;
import frc.robot.subsystems.*;

// TODO implement
public class FourBallAuton extends SequentialCommandGroup {

  private final AutoAim autoAimCommand;

  public FourBallAuton(
      DrivetrainSubsystem drivetrainSubsystem,
      TowerV2Subsystem towerSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem) {
    addCommands(
        new InstantCommand(towerSubsystem::intake, towerSubsystem),
        new FourBall0(drivetrainSubsystem),
        new ShootRpm(3000.0, towerSubsystem, shooterSubsystem).withTimeout(3.0),
        new InstantCommand(towerSubsystem::intake, towerSubsystem),
        new FourBall1(drivetrainSubsystem),
        new ShootRpm(4000.0, towerSubsystem, shooterSubsystem));

    autoAimCommand = new AutoAim(turretSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    autoAimCommand.schedule();

    super.initialize();
  }

  @Override
  public void cancel() {
    super.cancel();

    autoAimCommand.cancel();
  }
}
