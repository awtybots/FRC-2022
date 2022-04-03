// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FeedShooter extends RunCommand {

  private final TowerSubsystem tower;

  public FeedShooter(TowerSubsystem tower, ShooterSubsystem shooter, TurretSubsystem turret) {
    super(() -> tower.feed(turret.isAtTarget() && shooter.isAtTarget()), tower);
    this.tower = tower;
  }

  @Override
  public void end(boolean interrupted) {
    tower.stop();
  }
}
