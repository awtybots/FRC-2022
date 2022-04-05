package frc.robot.auton.blind;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.backup.SpinupRpmAndFeed;
import frc.robot.commands.main.IntakeAndIngest;
import frc.robot.subsystems.*;

public class OneAndOneAuton extends SequentialCommandGroup {

  public OneAndOneAuton(
      DrivetrainSubsystem drivetrain, TowerSubsystem tower, ShooterSubsystem shooter) {
    addCommands(
        new SpinupRpmAndFeed(1000, tower, shooter).withTimeout(4),
        new FunctionalCommand(
                () -> {},
                () -> drivetrain.driveVolts(3.0, 3.0),
                __ -> drivetrain.stop(),
                () -> false,
                drivetrain)
            .withTimeout(2.0)
            .alongWith(new IntakeAndIngest(tower).withTimeout(3.0)),
        new SpinupRpmAndFeed(2000, tower, shooter).withTimeout(5.0));
  }
}
