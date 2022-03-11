package frc.robot.commands.auton.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.backup.ShootRpm;
import frc.robot.subsystems.*;

public class OneBallAuton extends SequentialCommandGroup {
  public OneBallAuton(
      DrivetrainSubsystem drivetrainSubsystem,
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem) {
    addCommands(
        new StartEndCommand(
                () -> {
                  drivetrainSubsystem.drive(0.5, 0.0);
                },
                drivetrainSubsystem::stop,
                drivetrainSubsystem)
            .withTimeout(0.5), // TODO tune according to ZeroBallAuton
        new ShootRpm(2500, towerSubsystem, shooterSubsystem).withTimeout(5.0));
  }
}