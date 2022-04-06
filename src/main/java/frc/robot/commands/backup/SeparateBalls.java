// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TowerSubsystem;

public class SeparateBalls extends CommandBase {

  private final TowerSubsystem tower;

  public SeparateBalls(TowerSubsystem tower) {
    this.tower = tower;
  }

  @Override
  public void initialize() {
    tower.separateBalls();
  }

  @Override
  public void end(boolean interrupted) {
    tower.stop();
  }

  @Override
  public boolean isFinished() {
    return tower.isFull();
  }
}
