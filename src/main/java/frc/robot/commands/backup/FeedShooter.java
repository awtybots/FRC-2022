// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class FeedShooter extends CommandBase {

  private final TowerSubsystem tower;
  private final ShooterSubsystem shooter;

  public FeedShooter(TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem) {
    this.tower = towerSubsystem;
    this.shooter = shooterSubsystem;
    addRequirements(towerSubsystem);
  }

  @Override
  public void execute() {
    tower.feed(shooter.isAtTarget());
  }

  @Override
  public void end(boolean interrupted) {
    tower.stop();
  }
}
