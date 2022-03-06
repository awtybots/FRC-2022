package frc.robot.commands.auton.trajectories;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.subsystems.DrivetrainSubsystem;

public abstract class TrajectoryA extends DriveTrajectory {

  private static final Trajectory trajectory;

  static {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DrivetrainSubsystem.kTrajectoryMaxVelocity, DrivetrainSubsystem.kTrajectoryMaxAcceleration);
    trajectoryConfig.setKinematics(DrivetrainSubsystem.kKinematics);
    trajectoryConfig.addConstraint(new DifferentialDriveVoltageConstraint(DrivetrainSubsystem.kFeedforward, DrivetrainSubsystem.kKinematics, DrivetrainSubsystem.kMaxTrajectoryVoltage));
  
    trajectory = null;//TrajectoryGenerator.generateTrajectory(initial, interiorWaypoints, end, config)
  }

  public TrajectoryA(DrivetrainSubsystem drivetrainSubsystem) {
    super(trajectory, drivetrainSubsystem);
  }
}
