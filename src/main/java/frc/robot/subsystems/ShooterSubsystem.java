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

public class ShooterSubsystem extends SubsystemBase {

  private final WPI_TalonFX flywheel;

  public ShooterSubsystem() {
    flywheel = new WPI_TalonFX(Shooter.kFlywheelMotor);
    configMotors();
  }

  private void configMotors() {
    flywheel.configAllSettings(Shooter.motorConfig());
    flywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    flywheel.setInverted(true);
    flywheel.setNeutralMode(NeutralMode.Coast);
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
}
