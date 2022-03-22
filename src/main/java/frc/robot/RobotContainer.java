// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auton.blind.*;
import frc.robot.commands.backup.*;
import frc.robot.commands.main.*;
import frc.robot.subsystems.*;
import frc.util.Controller;

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

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    addAutonomousChoices();
    SmartDashboard.putData(autonChooser);
    CameraServer.startAutomaticCapture();
    configureButtonBindings();
  }

  private void addAutonomousChoices() {
    autonChooser.addOption("Do Nothing", new InstantCommand());
    autonChooser.addOption("Zero Ball Auton", new TaxiOffTarmacAuton(drivetrainSubsystem));
    autonChooser.addOption(
        "1 Low Goal 1 High Goal",
        new TwoBallLowAndHighAuton(
            drivetrainSubsystem,
            intakeSubsystem,
            towerSubsystem,
            turretSubsystem,
            shooterSubsystem,
            limelightSubsystem,
            colorSensorsSubsystem));
    autonChooser.setDefaultOption(
        "2 Ball High Goal",
        new TwoBallHighGoalAuton(
            drivetrainSubsystem,
            intakeSubsystem,
            towerSubsystem,
            turretSubsystem,
            shooterSubsystem,
            limelightSubsystem,
            colorSensorsSubsystem));
    // autonChooser.addOption(
    //     "One Ball Auton",
    //     new OneBallAuton(
    //         drivetrainSubsystem, towerSubsystem, shooterSubsystem, colorSensorsSubsystem));
    // autonChooser.addOption(
    //     "2 Ball Stupid",
    //     new TwoBallAutonStupid(
    //         drivetrainSubsystem,
    //         intakeSubsystem,
    //         towerSubsystem,
    //         turretSubsystem,
    //         shooterSubsystem,
    //         limelightSubsystem,
    //         colorSensorsSubsystem));
    // autonChooser.addOption(
    //     "Two Ball Auton",
    //     new TwoBallAuton(
    //         drivetrainSubsystem,
    //         intakeSubsystem,
    //         towerSubsystem,
    //         turretSubsystem,
    //         shooterSubsystem,
    //         limelightSubsystem,
    //         colorSensorsSubsystem));
    // autonChooser.addOption(
    //     "Four Ball Auton",
    //     new FourBallAuton(
    //         drivetrainSubsystem,
    //         intakeSubsystem,
    //         towerSubsystem,
    //         turretSubsystem,
    //         shooterSubsystem,
    //         limelightSubsystem,
    //         colorSensorsSubsystem));
  }

  private void configureButtonBindings() {
    // === DRIVER ===
    drivetrainSubsystem.setDefaultCommand(new Drive(driver, drivetrainSubsystem));
    driver.bumperRight.whenHeld(
        new IntakeAndIngest(intakeSubsystem, towerSubsystem, colorSensorsSubsystem));
    driver.bumperLeft.whenHeld(
        new ReverseIntake(intakeSubsystem).alongWith(new ReverseTower(towerSubsystem)));

    // === OPERATOR ===
    /// === AUTOMAGIC ===
    // operator.buttonBack.whenHeld(new AutoAim(turretSubsystem, limelightSubsystem));
    // operator.buttonStart.whenHeld(
    //     new ShootInterpolated(towerSubsystem, shooterSubsystem,
    // limelightSubsystem, colorSensorsSubsystem));

    /// === MANUAL ===
    operator.bumperLeft.whenHeld(new ReverseTower(towerSubsystem));
    climbSubsystem.setDefaultCommand(new DriveClimber(operator, climbSubsystem));

    /// === SHOOTING ===
    operator.buttonA.whenHeld(
        new ShootRpm(750, towerSubsystem, shooterSubsystem, colorSensorsSubsystem));
    operator.buttonB.whenHeld(
        new ShootRpm(1600, towerSubsystem, shooterSubsystem, colorSensorsSubsystem));
    operator.buttonX.whenHeld(
        new ShootRpm(1800, towerSubsystem, shooterSubsystem, colorSensorsSubsystem));
    operator.buttonY.whenHeld(
        new ShootRpm(2300, towerSubsystem, shooterSubsystem, colorSensorsSubsystem));

    /// === TURRET ===
    turretSubsystem.setDefaultCommand(new DriveTurret(operator, turretSubsystem));
    operator.dpadLeft.whileHeld(new TurnTurretTo(-90.0, turretSubsystem));
    operator.dpadUp.whileHeld(new TurnTurretTo(0.0, turretSubsystem));
    operator.dpadRight.whileHeld(new TurnTurretTo(90.0, turretSubsystem));
    operator.dpadDown.whileHeld(new TurnTurretTo(180.0, turretSubsystem));

    /// === PROGRAMMER TUNING ===
    // operator.buttonA.whenHeld(new ShootRpmSD(towerSubsystem, shooterSubsystem, colorSensorsSubsystem));
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

  public void turnOffLimelightLEDs() {
    limelightSubsystem.drivingMode();
  }
}
