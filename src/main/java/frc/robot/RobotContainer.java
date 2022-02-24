// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController controllerDriver = new XboxController(0);
  private final XboxController controllerOperator = new XboxController(1);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final TowerSubsystem towerSubsystem = new TowerSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

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
    drivetrainSubsystem.setDefaultCommand(new TeleopDrive(controllerDriver, drivetrainSubsystem));
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
