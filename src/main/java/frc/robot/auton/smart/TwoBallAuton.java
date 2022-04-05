package frc.robot.auton.smart;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auton.smart.trajectories.TwoBall0;
import frc.robot.commands.backup.SpinupRpmAndFeed;
import frc.robot.commands.main.AutoAim;
import frc.robot.commands.main.IntakeAndIngest;
import frc.robot.subsystems.*;

public class TwoBallAuton extends SequentialCommandGroup {

  private final AutoAim autoAimCommand;

  public TwoBallAuton(
      DrivetrainSubsystem drivetrain,
      TowerSubsystem tower,
      TurretSubsystem turret,
      ShooterSubsystem shooter) {
    addCommands(
        new TwoBall0(drivetrain).alongWith(new IntakeAndIngest(tower).withTimeout(5.0)),
        new SpinupRpmAndFeed(1950.0, tower, shooter).withTimeout(5.0));

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
