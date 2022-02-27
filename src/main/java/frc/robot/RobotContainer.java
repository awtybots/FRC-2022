// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveTurret;
import frc.robot.commands.Intake;
import frc.robot.commands.ShootBlind;
import frc.robot.subsystems.*;
import frc.robot.util.Controller;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Controller driver = new Controller(0);
  private final Controller operator = new Controller(1);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final TowerSubsystem towerSubsystem = new TowerSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final ColorSensorsSubsystem colorSensorsSubsystem = new ColorSensorsSubsystem();

  private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    addAutonomousChoices();
    configureButtonBindings();
  }

  private void addAutonomousChoices() {
    // autonChooser.setDefaultOption("name", command);
    // autonChooser.addOption("name", command);
    // autonChooser.addOption("name", command);
    // autonChooser.addOption("name", command);

    SmartDashboard.putData(autonChooser);
  }

  private void configureButtonBindings() {
    drivetrainSubsystem.setDefaultCommand(new Drive(driver, drivetrainSubsystem));
    driver.bumperRight.whenHeld(new Intake(intakeSubsystem, towerSubsystem, colorSensorsSubsystem));

    // NOTE: ShootBlind automatically runs tower once flywheel is at speed
    operator.buttonA.whenHeld(new ShootBlind(3600, towerSubsystem, shooterSubsystem));
    operator.buttonB.whenHeld(new ShootBlind(4100, towerSubsystem, shooterSubsystem));
    operator.buttonX.whenHeld(new ShootBlind(4600, towerSubsystem, shooterSubsystem));
    operator.buttonY.whenHeld(new ShootBlind(5700, towerSubsystem, shooterSubsystem));

    turretSubsystem.setDefaultCommand(new DriveTurret(operator, turretSubsystem));

    operator.bumperLeft.whenHeld(
        new StartEndCommand(towerSubsystem::reverseBoth, towerSubsystem::stop, towerSubsystem));
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
