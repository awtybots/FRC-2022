// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Tower;

public class TowerSubsystem extends SubsystemBase {

  private final WPI_TalonSRX upperMotor, lowerMotor;

  public TowerSubsystem() {
    upperMotor = new WPI_TalonSRX(Tower.kUpperMotor);
    lowerMotor = new WPI_TalonSRX(Tower.kLowerMotor);

    configMotors();
  }

  private void configMotors() {
    upperMotor.configAllSettings(Tower.motorConfig());
    lowerMotor.configAllSettings(Tower.motorConfig());

    upperMotor.setInverted(false);
    lowerMotor.setInverted(false);
  }
}
