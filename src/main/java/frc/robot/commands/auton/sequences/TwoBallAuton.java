package frc.robot.commands.auton.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.trajectories.TwoBall0;
import frc.robot.commands.backup.AutoAim;
import frc.robot.commands.backup.IntakeAndIngest;
import frc.robot.commands.backup.ShootRpm;
import frc.robot.subsystems.*;

public class TwoBallAuton extends SequentialCommandGroup {
  // private final DrivetrainSubsystem drivetrainSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  // private final TowerSubsystem towerSubsystem;
  private final TurretSubsystem turretSubsystem;
  // private final ShooterSubsystem shooterSubsystem;
  // private final LimelightSubsystem limelightSubsystem;

  private final AutoAim autoAimCommand;

  public TwoBallAuton(
      DrivetrainSubsystem drivetrainSubsystem,
      IntakeSubsystem intakeSubsystem,
      TowerSubsystem towerSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem,
      ColorSensorsSubsystem colorSensorsSubsystem) {
    addCommands(
        new TwoBall0(drivetrainSubsystem).alongWith(new IntakeAndIngest(intakeSubsystem, towerSubsystem, colorSensorsSubsystem).withTimeout(5.0)),
        new ShootRpm(1950.0, towerSubsystem, shooterSubsystem).withTimeout(5.0)
        );

    autoAimCommand = new AutoAim(turretSubsystem, limelightSubsystem);

    // this.drivetrainSubsystem = drivetrainSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    // this.towerSubsystem = towerSubsystem;
    this.turretSubsystem = turretSubsystem;
    // this.shooterSubsystem = shooterSubsystem;
    // this.limelightSubsystem = limelightSubsystem;
  }

  @Override
  public void initialize() {
    turretSubsystem.initPosition(180.0);
    autoAimCommand.schedule();

    super.initialize();
  }

  @Override
  public void cancel() {
    super.cancel();

    autoAimCommand.cancel();
    intakeSubsystem.stop();
  }
}
