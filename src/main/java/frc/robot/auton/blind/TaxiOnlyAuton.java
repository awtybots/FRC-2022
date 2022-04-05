package frc.robot.auton.blind;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TaxiOnlyAuton extends SequentialCommandGroup {
  public TaxiOnlyAuton(DrivetrainSubsystem drivetrain) {
    addCommands(
        new FunctionalCommand(
                () -> {},
                () -> drivetrain.driveVolts(3.0, 3.0),
                __ -> drivetrain.stop(),
                () -> false,
                drivetrain)
            .withTimeout(2.0));
  }
}
