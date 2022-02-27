// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import frc.robot.util.math.Convert;
import frc.robot.util.math.Convert.Encoder;

public class ShooterSubsystem extends SubsystemBase {

  private final WPI_TalonFX flywheel;
  private final double kMaxFlywheelRPM = 5600; // ! FIXME! find correct max flywheel rpm
  private final double kMaxAcceptableRPMError = 150.0; // TODO! we can do better
  private final double kF_Flywheel = calculateKF(5500, 0.80);
  private final double kP_Flywheel = 0.0;

  public ShooterSubsystem() {
    flywheel = new WPI_TalonFX(Shooter.kFlywheelMotor);
    configMotors();
  }

  private void configMotors() {
    flywheel.configAllSettings(Shooter.motorConfig());
    flywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    flywheel.setInverted(true); // note: no need to adjust sensorPhase for Falcons
    flywheel.setNeutralMode(NeutralMode.Coast);

    flywheel.config_kF(0, kF_Flywheel);
    flywheel.config_kP(0, kP_Flywheel);
  }

  /**
   * To calculate the parameters for this function
   *
   * <ol>
   *   <li>Set the flywheel output to a percent output that is the average for most shots
   *   <li>Let it reach a steady state
   *   <li>Record the percent output and steady state rpm
   *   <li>Pass those values into this function to calculate the feedforward gain.
   * </ol>
   */
  private double calculateKF(double rpmAtPercentOut, double percentOut) {
    // see the following link for an explanation of the math below
    // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#how-to-calculate-kf
    double sensorVelAtPercentOut =
        Shooter.kFlywheelRatio
            * Convert.rpmToEncoderVel(rpmAtPercentOut, Encoder.TalonFXIntegrated);
    return (percentOut * 1023) / sensorVelAtPercentOut;
  }

  public void percentShoot(double percent) {
    flywheel.set(ControlMode.PercentOutput, percent);
  }

  public void shoot(double rpm) {
    flywheel.set(ControlMode.Velocity, clampToBounds(rpm));
  }

  public void stop() {
    flywheel.set(ControlMode.PercentOutput, 0);
  }

  private final double clampToBounds(double rpm) {
    if (rpm > 0 && rpm <= kMaxFlywheelRPM) return rpm;
    else if ((rpm <= 1E-2 && rpm >= 1E-2) || rpm < 0) return 0;
    else if (rpm > kMaxFlywheelRPM) return kMaxFlywheelRPM;
    else {
      System.err.println(String.format("ERROR Clamping Flywheel RPM of %s", rpm));
      return 0;
    }
  }

  private boolean withinBounds(double target, double maxDev, double value) {
    return (value <= (target + maxDev)) && (value >= (target - maxDev));
  }

  public boolean atTarget(double target) {
    return withinBounds(
        target,
        kMaxAcceptableRPMError,
        Convert.encoderVelToRPM(flywheel.getSelectedSensorVelocity(), Encoder.TalonFXIntegrated));
  }
}
