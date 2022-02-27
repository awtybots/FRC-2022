// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import frc.robot.util.math.Convert;
import frc.robot.util.math.Convert.Encoder;

public class ShooterSubsystem extends SubsystemBase {

  private final double kP = 0;
  private final double kI = 0;
  private final double kD = 0;
  private final double kF = 0;
  
  private final double kFlywheelRpmThreshold = 50.0; // TODO

  private final WPI_TalonFX flywheel;

  private double goalRpm = 0.0;
  private double actualRpm = 0.0;

  public ShooterSubsystem() {
    flywheel = new WPI_TalonFX(Shooter.kFlywheelMotor);
    configMotors();
  }

  private void configMotors() {
    flywheel.configVoltageCompSaturation(12.0);
    flywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    flywheel.config_kP(0, kP);
    flywheel.config_kI(0, kI);
    flywheel.config_kD(0, kD);
    flywheel.config_kF(0, kF);
  }

  @Override
  public void periodic() {
    actualRpm = getRpm();

    SmartDashboard.putNumber("SH - actual rpm", actualRpm);
    SmartDashboard.putNumber("SH - goal rpm", goalRpm);
    SmartDashboard.putBoolean("SH - at goal", isRpmAtGoal());
  }

  public void setRpm(double rpm) {
    flywheel.set(ControlMode.Velocity, Convert.rpmToEncoderVel(rpm, Encoder.TalonFXIntegrated));
  }

  private double getRpm() {
    return Convert.encoderVelToRpm(flywheel.getSelectedSensorVelocity(), Encoder.TalonFXIntegrated);
  }

  public boolean isRpmAtGoal() {
    return Math.abs(actualRpm - goalRpm) < kFlywheelRpmThreshold;
  }

  public void stop() {
    flywheel.set(ControlMode.PercentOutput, 0.0);
  }
}
