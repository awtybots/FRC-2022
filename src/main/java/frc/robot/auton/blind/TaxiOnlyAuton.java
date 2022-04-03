package frc.robot.auton.blind;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TaxiOnlyAuton extends SequentialCommandGroup {
  public TaxiOnlyAuton(DrivetrainSubsystem drivetrainSubsystem) {
    addCommands(
        new FunctionalCommand(
                () -> {},
                () -> drivetrainSubsystem.driveVolts(3.0, 3.0),
                interrupted -> drivetrainSubsystem.stop(),
                () -> false,
                drivetrainSubsystem)
            .withTimeout(2.0));
  }
}
