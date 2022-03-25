package frc.robot.auton.blind;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.backup.ShootRpm;
import frc.robot.commands.main.AutoAim;
import frc.robot.commands.main.IntakeAndIngest;
import frc.robot.subsystems.*;

// this is good
public class TwoBallHighGoalAuton extends SequentialCommandGroup {

  public TwoBallHighGoalAuton(
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
                  drivetrainSubsystem.driveVolts(3.0, 3.0);
                },
                interrupted -> {
                  drivetrainSubsystem.stop();
                },
                () -> false,
                drivetrainSubsystem)
            .withTimeout(2.0)
            .alongWith(
                new IntakeAndIngest(intakeSubsystem, towerSubsystem, colorSensorsSubsystem)
                    .withTimeout(3.0)),
        new ShootRpm(2000, towerSubsystem, shooterSubsystem, colorSensorsSubsystem)
            .alongWith(new AutoAim(turretSubsystem, limelightSubsystem))
            .withTimeout(5.0));
  }
}
