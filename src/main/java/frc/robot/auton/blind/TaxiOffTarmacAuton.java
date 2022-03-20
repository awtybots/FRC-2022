package frc.robot.auton.blind;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TaxiOffTarmacAuton extends SequentialCommandGroup {
  public TaxiOffTarmacAuton(DrivetrainSubsystem drivetrainSubsystem) {
    addCommands(
        new StartEndCommand( // TODO less violent
                () -> {
                  drivetrainSubsystem.driveVolts(6.0, 6.0);
                },
                drivetrainSubsystem::stop,
                drivetrainSubsystem)
            .withTimeout(0.5) // TODO tune this auton first
        );
  }
}
