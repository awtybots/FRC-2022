package frc.robot.auton.blind;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.backup.SpinupRpmAndFeed;
import frc.robot.commands.main.AutoAim;
import frc.robot.commands.main.IntakeAndIngest;
import frc.robot.subsystems.*;

public class FourBallRightSide extends SequentialCommandGroup {

  public FourBallRightSide(
      DrivetrainSubsystem drivetrain,
      TowerSubsystem tower,
      TurretSubsystem turret,
      ShooterSubsystem shooter) {
    addCommands(
        new FunctionalCommand(
                () -> {},
                () -> drivetrain.driveVolts(3.0, 3.0),
                __ -> drivetrain.stop(),
                () -> false,
                drivetrain)
            .alongWith(new IntakeAndIngest(tower))
            .withTimeout(3.0),
        new AutoAim(turret).withTimeout(2.0),
        new SpinupRpmAndFeed(1950, tower, shooter).withTimeout(3.0),
        new FunctionalCommand(
                () -> {},
                () -> drivetrain.driveVolts(3.0, 3.0),
                __ -> drivetrain.stop(),
                () -> false,
                drivetrain)
            .alongWith(new IntakeAndIngest(tower))
            .withTimeout(4.0),
        new IntakeAndIngest(tower).withTimeout(1.0),
        new FunctionalCommand(
                () -> {},
                () -> drivetrain.driveVolts(-3.0, -3.0),
                __ -> drivetrain.stop(),
                () -> false,
                drivetrain)
            .alongWith(new IntakeAndIngest(tower))
            .withTimeout(4.5),
        new SpinupRpmAndFeed(1950, tower, shooter).withTimeout(3.0));
  }
}
