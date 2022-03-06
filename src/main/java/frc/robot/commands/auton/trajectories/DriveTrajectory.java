package frc.robot.commands.auton.trajectories;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public abstract class DriveTrajectory extends RamseteCommand {

  private final Trajectory trajectory;
  private final DrivetrainSubsystem drivetrainSubsystem;

  DriveTrajectory(Trajectory trajectory, DrivetrainSubsystem drivetrainSubsystem) {
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

  @Override
  public void initialize() {
    drivetrainSubsystem.initOdometry(trajectory.getInitialPose());
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrainSubsystem.driveVolts(0.0, 0.0);
  }
}
