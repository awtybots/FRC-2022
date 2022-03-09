package frc.robot.commands.auton.sequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.ShootRpmNoIntake;
import frc.robot.commands.auton.trajectories.FourBall0;
import frc.robot.commands.auton.trajectories.FourBall1;
import frc.robot.commands.backup.AutoAim;
import frc.robot.subsystems.*;

public class FourBallAuton extends SequentialCommandGroup {
  // private final DrivetrainSubsystem drivetrainSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  // private final TowerSubsystem towerSubsystem;
  private final TurretSubsystem turretSubsystem;
  // private final ShooterSubsystem shooterSubsystem;
  // private final LimelightSubsystem limelightSubsystem;

  private final AutoAim autoAimCommand;

  public FourBallAuton(
      DrivetrainSubsystem drivetrainSubsystem,
      IntakeSubsystem intakeSubsystem,
      TowerSubsystem towerSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem) {
    addCommands(
        new InstantCommand(intakeSubsystem::start, intakeSubsystem),
        new FourBall0(drivetrainSubsystem),
        new ShootRpmNoIntake(3000.0, towerSubsystem, shooterSubsystem) // TODO tune rpm and time
            .withTimeout(3.0),
        new FourBall1(drivetrainSubsystem),
        new ShootRpmNoIntake(4000.0, towerSubsystem, shooterSubsystem));

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

    intakeSubsystem.stop();
    autoAimCommand.cancel();
  }
}
