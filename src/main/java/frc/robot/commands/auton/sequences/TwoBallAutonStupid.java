package frc.robot.commands.auton.sequences;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.backup.IntakeAndIngest;
import frc.robot.commands.backup.ShootRpm;
import frc.robot.subsystems.*;

public class TwoBallAutonStupid extends SequentialCommandGroup {

  public TwoBallAutonStupid(
      DrivetrainSubsystem drivetrainSubsystem,
      IntakeSubsystem intakeSubsystem,
      TowerSubsystem towerSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem,
      ColorSensorsSubsystem colorSensorsSubsystem) {
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
            .withTimeout(1.0).alongWith(
              new IntakeAndIngest(intakeSubsystem, towerSubsystem, colorSensorsSubsystem).withTimeout(5.0)
              ),
          new ShootRpm(1750, towerSubsystem, shooterSubsystem).withTimeout(5.0)
        );
  }
}
