// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.backup.*;
import frc.robot.commands.main.*;
import frc.robot.subsystems.*;
import frc.robot.util.Controller;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static PowerDistribution pdp;

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

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    initElectronics();
    addAutonomousChoices();
    configureButtonBindings();
  }

  private void initElectronics() {
    pdp = new PowerDistribution();
  }

  private void addAutonomousChoices() {
    // autonChooser.setDefaultOption("name", command);
    // autonChooser.addOption("name", command);
    // autonChooser.addOption("name", command);
    // autonChooser.addOption("name", command);

    SmartDashboard.putData(autonChooser);
  }

  private void configureButtonBindings() {
    // === AUTO ===
    // shooterSubsystem.setDefaultCommand(
    //     new MovingShots(
    //         towerSubsystem,
    //         shooterSubsystem,
    //         turretSubsystem,
    //         drivetrainSubsystem,
    //         colorSensorsSubsystem,
    //         limelightSubsystem));
    // shooterSubsystem.setDefaultCommand(new IdleShooter(shooterSubsystem));
    // turretSubsystem.setDefaultCommand(new AutoAim(turretSubsystem, limelightSubsystem));

    // === DRIVER ===
    drivetrainSubsystem.setDefaultCommand(new Drive(driver, drivetrainSubsystem));
    // driver.bumperRight.whenHeld(new Intake(intakeSubsystem, towerSubsystem,
    // colorSensorsSubsystem));
    driver.bumperRight.whenHeld(
        new IntakeAndIngest(intakeSubsystem, towerSubsystem, colorSensorsSubsystem));

    // === OPERATOR ===
    // turretSubsystem.setDefaultCommand(new DriveTurret(operator, turretSubsystem));
    climbSubsystem.setDefaultCommand(new DriveClimber(operator, climbSubsystem)); // TODO temp

    operator.buttonA.whenHeld(
        new ShootRpmOrSpit(
            1000,
            towerSubsystem,
            shooterSubsystem,
            turretSubsystem,
            limelightSubsystem,
            colorSensorsSubsystem));
    operator.buttonB.whenHeld(
        new ShootRpmOrSpit(
            1500,
            towerSubsystem,
            shooterSubsystem,
            turretSubsystem,
            limelightSubsystem,
            colorSensorsSubsystem));
    operator.buttonX.whenHeld(
        new ShootRpmOrSpit(
            2300,
            towerSubsystem,
            shooterSubsystem,
            turretSubsystem,
            limelightSubsystem,
            colorSensorsSubsystem));
    operator.buttonY.whenHeld(
        new ShootRpmOrSpit(
            3000,
            towerSubsystem,
            shooterSubsystem,
            turretSubsystem,
            limelightSubsystem,
            colorSensorsSubsystem));

    operator.bumperLeft.whenHeld(new ReverseTower(towerSubsystem));
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
