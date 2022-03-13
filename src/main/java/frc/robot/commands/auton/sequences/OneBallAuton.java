package frc.robot.commands.auton.sequences;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.backup.ShootRpm;
import frc.robot.subsystems.*;

public class OneBallAuton extends SequentialCommandGroup {
  public OneBallAuton(
      DrivetrainSubsystem drivetrainSubsystem,
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem) {
    addCommands(
        new FunctionalCommand(
                () -> {},
                () -> {
                  drivetrainSubsystem.driveVolts(6.0, 6.0);
                },
                interrupted -> {
                  drivetrainSubsystem.stop();
                },
                () -> false,
                drivetrainSubsystem)
            .withTimeout(1.0),
        new ShootRpm(1750, towerSubsystem, shooterSubsystem).withTimeout(5.0));
  }
}
