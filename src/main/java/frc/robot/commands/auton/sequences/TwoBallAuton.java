package frc.robot.commands.auton.sequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.trajectories.TwoBall0;
import frc.robot.commands.backup.ShootRpm;
import frc.robot.subsystems.*;

public class TwoBallAuton extends SequentialCommandGroup {
  // private final DrivetrainSubsystem drivetrainSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  // private final TowerSubsystem towerSubsystem;
  private final TurretSubsystem turretSubsystem;
  // private final ShooterSubsystem shooterSubsystem;
  // private final LimelightSubsystem limelightSubsystem;

  public TwoBallAuton(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem, TowerSubsystem towerSubsystem, TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    addCommands(
      new InstantCommand(intakeSubsystem::start, intakeSubsystem),
      new TwoBall0(drivetrainSubsystem),
      new ShootRpm(3000.0, towerSubsystem, shooterSubsystem)
      // .alongWith(new AutoAim(turretSubsystem, limelightSubsystem))
    );

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

    super.initialize();
  }

  @Override
  public void cancel() {
    super.cancel();

    intakeSubsystem.stop();
  }
}
