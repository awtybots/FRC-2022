package frc.robot.commands.auton.trajectories;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.subsystems.DrivetrainSubsystem;

public abstract class TrajectoryA extends DriveTrajectory {

  private static final Trajectory trajectory;

  static {
    trajectory = new Trajectory(); // TODO this is an example
  }

  public TrajectoryA(DrivetrainSubsystem drivetrainSubsystem) {
    super(trajectory, drivetrainSubsystem);
  }
}
