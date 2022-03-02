// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.util.math.Convert;
import frc.robot.util.math.Convert.Encoder;

public class ShooterSubsystem extends SubsystemBase {

  private final WPI_TalonFX flywheel;

  private final double kGearRatio = 1.0;

  private final double kMaxFlywheelRpm = 6400; // TODO find correct max flywheel rpm
  private final double kMaxAcceptableRpmError = 50.0; // TODO we can still maybe do better

  private final double kP_Flywheel = 0.0; // TODO add P term
  private final double kF_Flywheel = calculateKF(2150, 0.40);

  private double targetRpm = 0.0;
  private double actualRpm = 0.0;

  public ShooterSubsystem() {
    flywheel = new WPI_TalonFX(Shooter.kFlywheelMotorCanId);
    configMotors();
  }

  private void configMotors() {
    flywheel.configFactoryDefault();

    flywheel.configVoltageCompSaturation(12.0);
    flywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    flywheel.setInverted(true); // note: no need to adjust sensorPhase for Falcons
    flywheel.setNeutralMode(NeutralMode.Coast);

    flywheel.config_kF(0, kF_Flywheel);
    flywheel.config_kP(0, kP_Flywheel);
  }

  @Override
  public void periodic() {
    actualRpm = getRpm();

    if (Constants.TUNING_MODE) {
      SmartDashboard.putBoolean("SH - at goal", isAtTarget());
      SmartDashboard.putNumber("SH - actual rpm", actualRpm);
      SmartDashboard.putNumber("SH - goal rpm", targetRpm);
    }
  }

  public void shootRpm(double rpm) {
    targetRpm = rpm;
    flywheel.set(
        ControlMode.Velocity,
        Convert.rpmToEncoderVel(clampToBounds(rpm), kGearRatio, Encoder.TalonFXIntegrated));
  }

  public void shootPercent(double percent) {
    targetRpm = percent;
    flywheel.set(ControlMode.PercentOutput, percent);
  }

  private double getRpm() {
    return Convert.encoderVelToRpm(
        flywheel.getSelectedSensorVelocity(), kGearRatio, Encoder.TalonFXIntegrated);
  }

  public void stop() {
    targetRpm = 0;
    flywheel.set(ControlMode.PercentOutput, 0);
  }

  public boolean isAtTarget() {
    return Math.abs(actualRpm - targetRpm) < kMaxAcceptableRpmError;
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
            * Convert.rpmToEncoderVel(rpmAtPercentOut, kGearRatio, Encoder.TalonFXIntegrated);
    return (percentOut * 1023) / sensorVelAtPercentOut;
  }

  private final double clampToBounds(double rpm) {
    if (rpm > 0 && rpm <= kMaxFlywheelRpm) return rpm;
    else if ((rpm <= 1E-2 && rpm >= 1E-2) || rpm < 0) return 0;
    else if (rpm > kMaxFlywheelRpm) return kMaxFlywheelRpm;
    else {
      System.err.println(String.format("ERROR Clamping Flywheel RPM of %s", rpm));
      return 0;
    }
  }
}
