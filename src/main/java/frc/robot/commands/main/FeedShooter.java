// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FeedShooter extends CommandBase {

  private final TowerSubsystem tower;
  private final ShooterSubsystem shooter;
  private final TurretSubsystem turret;

  public FeedShooter(TowerSubsystem tower, ShooterSubsystem shooter, TurretSubsystem turret) {
    this.tower = tower;
    this.shooter = shooter;
    this.turret = turret;

    addRequirements(tower);
  }

  @Override
  public void initialize() {
    RobotContainer.ledSubsystem.blink();
  }

  @Override
  public void execute() {
    tower.feed(turret.isAtTarget() && shooter.isAtTarget());
  }

  @Override
  public void end(boolean interrupted) {
    tower.stop();
    RobotContainer.ledSubsystem.turnOn();
  }
}
