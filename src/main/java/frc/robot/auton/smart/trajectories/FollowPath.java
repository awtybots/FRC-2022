// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.smart.trajectories;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public abstract class FollowPath extends RamseteCommand {

  private final Trajectory trajectory;
  private final DrivetrainSubsystem drivetrainSubsystem;

  FollowPath(Trajectory trajectory, DrivetrainSubsystem drivetrainSubsystem) {
    super(
        trajectory,
        drivetrainSubsystem::getPose,
        new RamseteController(),
        DrivetrainSubsystem.kFeedforward,
        DrivetrainSubsystem.kKinematics,
        drivetrainSubsystem::getWheelSpeeds,
        new PIDController(DrivetrainSubsystem.kP, 0.0, 0.0),
        new PIDController(DrivetrainSubsystem.kP, 0.0, 0.0),
        drivetrainSubsystem::driveVolts,
        drivetrainSubsystem);

    this.trajectory = trajectory;
    this.drivetrainSubsystem = drivetrainSubsystem;
  }

  FollowPath(String fileName, DrivetrainSubsystem drivetrainSubsystem) {
    this(loadTrajectoryFromFile(fileName), drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    drivetrainSubsystem.initOdometry(trajectory.getInitialPose());
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrainSubsystem.stop();
  }

  private static Trajectory loadTrajectoryFromFile(String pathName) {
    return PathPlanner.loadPath(pathName, 3, 2);
  }
}
