package frc.robot.auton.blind;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auton.blind.segments.DrivePercent;
import frc.robot.commands.backup.ShootRpm;
import frc.robot.commands.main.AutoAim;
import frc.robot.commands.main.IntakeAndIngest;
import frc.robot.subsystems.*;

public class FourBallRightSide extends SequentialCommandGroup {

    public FourBallRightSide(
            DrivetrainSubsystem drivetrainSubsystem,
            TowerSubsystem towerSubsystem,
            TurretSubsystem turretSubsystem,
            ShooterSubsystem shooterSubsystem) {
        addCommands(
                new DrivePercent(1.0, 1.0, drivetrainSubsystem)
                        .alongWith(new IntakeAndIngest(towerSubsystem))
                        .alongWith(new AutoAim(turretSubsystem))
                        .withTimeout(1.0),
                new AutoAim(turretSubsystem).withTimeout(1.0),
                new ShootRpm(1950, towerSubsystem, shooterSubsystem).withTimeout(3.0),
                new DrivePercent(1.0, 1.0, drivetrainSubsystem)
                        .alongWith(new IntakeAndIngest(towerSubsystem))
                        .withTimeout(1.0),
                new IntakeAndIngest(towerSubsystem).withTimeout(1.0),
                new DrivePercent(-1.0, -1.0, drivetrainSubsystem)
                        .alongWith(new IntakeAndIngest(towerSubsystem))
                        .withTimeout(1.0),
                new ShootRpm(1950, towerSubsystem, shooterSubsystem).withTimeout(3.0));
    }
}
