package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class ActuateTraversalPistons extends InstantCommand {
    public ActuateTraversalPistons(ClimbSubsystem climber) {
        super(climber::togglePistons, climber);
    }
}
