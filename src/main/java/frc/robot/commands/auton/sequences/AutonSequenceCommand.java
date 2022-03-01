package frc.robot.commands.auton.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public abstract class AutonSequenceCommand extends SequentialCommandGroup {
  public Pose2d getInitialPose() {
    return new Pose2d();
  }
}
