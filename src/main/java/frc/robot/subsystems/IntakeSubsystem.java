// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {

  private final DoubleSolenoid pistons;
  private final WPI_TalonSRX motor;

  public IntakeSubsystem() {
    pistons =
        new DoubleSolenoid(PneumaticsModuleType.REVPH, Intake.kSolenoidDown, Intake.kSolenoidUp);
    motor = new WPI_TalonSRX(Intake.kMotor);
    configMotors();
  }

  private void configMotors() {
    motor.configAllSettings(Intake.motorConfig());
  }
}
