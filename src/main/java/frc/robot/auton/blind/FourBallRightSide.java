package frc.robot.auton.blind;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.backup.ShootRpm;
import frc.robot.commands.main.AutoAim;
import frc.robot.commands.main.IntakeAndIngest;
import frc.robot.subsystems.*;

public class FourBallRightSide extends SequentialCommandGroup {

  public FourBallRightSide(
      DrivetrainSubsystem drivetrainSubsystem,
      TowerV2Subsystem towerSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem) {
    addCommands(
        new FunctionalCommand(
                () -> {},
                () -> {
                  drivetrainSubsystem.driveVolts(3.0, 3.0);
                },
                interrupted -> {
                  drivetrainSubsystem.stop();
                },
                () -> false,
                drivetrainSubsystem)
            .alongWith(new IntakeAndIngest(towerSubsystem))
            .withTimeout(3.0),
        new AutoAim(turretSubsystem, limelightSubsystem).withTimeout(2.0),
        new ShootRpm(1950, towerSubsystem, shooterSubsystem).withTimeout(3.0),
        new FunctionalCommand(
                () -> {},
                () -> {
                  drivetrainSubsystem.driveVolts(3.0, 3.0);
                },
                interrupted -> {
                  drivetrainSubsystem.stop();
                },
                () -> false,
                drivetrainSubsystem)
            .alongWith(new IntakeAndIngest(towerSubsystem))
            .withTimeout(4.0),
        new IntakeAndIngest(towerSubsystem).withTimeout(1.0),
        new FunctionalCommand(
                () -> {},
                () -> {
                  drivetrainSubsystem.driveVolts(-3.0, -3.0);
                },
                interrupted -> {
                  drivetrainSubsystem.stop();
                },
                () -> false,
                drivetrainSubsystem)
            .alongWith(new IntakeAndIngest(towerSubsystem))
            .withTimeout(4.5),
        new ShootRpm(1950, towerSubsystem, shooterSubsystem).withTimeout(3.0));
  }
}
