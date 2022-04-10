package frc.robot.auton.blind.segments;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/** does not end on its own */
public class DrivePercent extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final double voltsL, voltsR;

  public DrivePercent(double pctL, double pctR, DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.voltsL = pctL * 12.0;
    this.voltsR = pctR * 12.0;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivetrainSubsystem.driveVolts(voltsL, voltsR);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
