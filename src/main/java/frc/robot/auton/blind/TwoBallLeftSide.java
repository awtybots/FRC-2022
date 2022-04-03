package frc.robot.auton.blind;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.backup.ShootRpm;
import frc.robot.commands.main.IntakeAndIngest;
import frc.robot.subsystems.*;

public class TwoBallLeftSide extends SequentialCommandGroup {

  public TwoBallLeftSide(
      DrivetrainSubsystem drivetrainSubsystem,
      TowerSubsystem towerSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem) {
    addCommands(
        new FunctionalCommand(
                () -> {},
                () -> drivetrainSubsystem.driveVolts(3.0, 3.0),
                interrupted -> drivetrainSubsystem.stop(),
                () -> false,
                drivetrainSubsystem)
            .withTimeout(2.0)
            .alongWith(new IntakeAndIngest(towerSubsystem).withTimeout(3.0)),
        new ShootRpm(1750, towerSubsystem, shooterSubsystem).withTimeout(5.0));
  }
}
