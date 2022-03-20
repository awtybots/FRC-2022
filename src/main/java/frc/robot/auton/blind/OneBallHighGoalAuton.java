package frc.robot.auton.blind;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.backup.ShootRpm;
import frc.robot.subsystems.*;

public class OneBallHighGoalAuton extends SequentialCommandGroup {
  public OneBallHighGoalAuton(
      DrivetrainSubsystem drivetrainSubsystem,
      TowerSubsystem towerSubsystem,
      ShooterSubsystem shooterSubsystem,
      ColorSensorsSubsystem colorSensorsSubsystem) {
    addCommands(
        new FunctionalCommand( // TODO replace with StartEndCommand
                () -> {},
                () -> {
                  drivetrainSubsystem.driveVolts(3.0, 3.0);
                },
                interrupted -> {
                  drivetrainSubsystem.stop();
                },
                () -> false,
                drivetrainSubsystem)
            .withTimeout(2.0),
        new ShootRpm(1950, towerSubsystem, shooterSubsystem, colorSensorsSubsystem)
            .withTimeout(5.0));
  }
}