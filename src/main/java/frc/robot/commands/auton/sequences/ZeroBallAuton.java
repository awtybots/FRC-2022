package frc.robot.commands.auton.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ZeroBallAuton extends SequentialCommandGroup {
  public ZeroBallAuton(DrivetrainSubsystem drivetrainSubsystem) {
    addCommands(
        new StartEndCommand(
                () -> {
                  drivetrainSubsystem.drive(0.5, 0.0);
                },
                drivetrainSubsystem::stop,
                drivetrainSubsystem)
            .withTimeout(0.5) // TODO tune this auton first
        );
  }
}
