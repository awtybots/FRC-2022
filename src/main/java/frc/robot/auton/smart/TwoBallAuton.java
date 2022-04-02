package frc.robot.auton.smart;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auton.trajectories.TwoBall0;
import frc.robot.commands.backup.ShootRpm;
import frc.robot.commands.main.AutoAim;
import frc.robot.commands.main.IntakeAndIngest;
import frc.robot.subsystems.*;

public class TwoBallAuton extends SequentialCommandGroup {

  private final AutoAim autoAimCommand;

  public TwoBallAuton(
      DrivetrainSubsystem drivetrainSubsystem,
      TowerSubsystem towerSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem) {
    addCommands(
        new TwoBall0(drivetrainSubsystem)
            .alongWith(new IntakeAndIngest(towerSubsystem).withTimeout(5.0)),
        new ShootRpm(1950.0, towerSubsystem, shooterSubsystem).withTimeout(5.0));

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
