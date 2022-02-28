package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class RetractClimb extends FunctionalCommand {

  public RetractClimb(ClimbSubsystem climbSubsystem) {
    super(
        climbSubsystem::retractClimb,
        () -> {},
        (interrupted) -> {
          climbSubsystem.stop();
        },
        climbSubsystem::isAtTarget,
        climbSubsystem);
  }
}
