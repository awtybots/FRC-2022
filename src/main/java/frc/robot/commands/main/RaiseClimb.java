package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class RaiseClimb extends FunctionalCommand {

  public RaiseClimb(ClimbSubsystem climbSubsystem) {
    super(
        climbSubsystem::raiseClimb,
        () -> {},
        (interrupted) -> {
          climbSubsystem.stop();
        },
        climbSubsystem::isAtTarget,
        climbSubsystem);
  }
}
