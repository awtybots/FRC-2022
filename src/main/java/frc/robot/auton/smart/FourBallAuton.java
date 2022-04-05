package frc.robot.auton.smart;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auton.smart.trajectories.FourBall0;
import frc.robot.auton.smart.trajectories.FourBall1;
import frc.robot.commands.backup.SpinupRpmAndFeed;
import frc.robot.commands.main.AutoAim;
import frc.robot.subsystems.*;

public class FourBallAuton extends SequentialCommandGroup {

  private final AutoAim autoAimCommand;

  public FourBallAuton(
      DrivetrainSubsystem drivetrain,
      TowerSubsystem tower,
      TurretSubsystem turret,
      ShooterSubsystem shooter) {
    addCommands(
        new InstantCommand(tower::intake, tower),
        new FourBall0(drivetrain),
        new SpinupRpmAndFeed(3000.0, tower, shooter).withTimeout(3.0),
        new InstantCommand(tower::intake, tower),
        new FourBall1(drivetrain),
        new SpinupRpmAndFeed(4000.0, tower, shooter));

    autoAimCommand = new AutoAim(turret);
  }

  @Override
  public void initialize() {
    autoAimCommand.schedule();

    super.initialize();
  }

  @Override
  public void cancel() {
    super.cancel();

    autoAimCommand.cancel();
  }
}
