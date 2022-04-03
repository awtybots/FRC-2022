package frc.robot.auton.smart.trajectories;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.io.IOException;
import java.nio.file.Path;

public abstract class DrivePathweaverTrajectory extends RamseteCommand {

  private final Trajectory trajectory;
  private final DrivetrainSubsystem drivetrainSubsystem;

  DrivePathweaverTrajectory(Trajectory trajectory, DrivetrainSubsystem drivetrainSubsystem) {
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

  DrivePathweaverTrajectory(String fileName, DrivetrainSubsystem drivetrainSubsystem) {
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
    try {
      Path trajectoryPath =
          Filesystem.getDeployDirectory().toPath().resolve("paths/" + pathName + ".wpilib.json");
      Trajectory t = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      return t;
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + pathName, ex.getStackTrace());
      return new Trajectory();
    }
  }
}
