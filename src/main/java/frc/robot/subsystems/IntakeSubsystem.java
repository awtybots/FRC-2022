// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {

  private final DoubleSolenoid pistons;
  private final WPI_TalonSRX motor;

  private final double kMotorSpeed = 0.75; // FIXME determine appropriate intake motor speed

  public IntakeSubsystem() {
    pistons =
        new DoubleSolenoid(PneumaticsModuleType.REVPH, Intake.kSolenoidDown, Intake.kSolenoidUp);
    motor = new WPI_TalonSRX(Intake.kMotorCanId);
    configMotors();

    stop();
  }

  private void configMotors() {
    motor.configFactoryDefault();
  }

  public void start() {
    pistons.set(Value.kForward);
    motor.set(ControlMode.PercentOutput, kMotorSpeed);
  }

  public void stop() {
    pistons.set(Value.kReverse);
    motor.set(ControlMode.PercentOutput, 0.0);
  }
}
