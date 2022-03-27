// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auton.blind.*;
import frc.robot.commands.backup.*;
import frc.robot.commands.main.*;
import frc.robot.subsystems.*;
import frc.util.AutonManager;
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

  // public static final PowerDistribution pdh = new PowerDistribution(0, ModuleType.kRev);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final TowerSubsystem towerSubsystem = new TowerSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  public static final LedSubsystem ledSubsystem = new LedSubsystem();

  // private final SendableChooser<Command> autonChooser = new SendableChooser<>();
  private final AutonManager autonManager = new AutonManager();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    addAutonomousChoices();
    autonManager.displayChoices();

    configureButtonBindings();

    miscellaneousStartup();
  }

  private void miscellaneousStartup() {
    LiveWindow.disableAllTelemetry();
    CameraServer.startAutomaticCapture();
  }

  private void addAutonomousChoices() {
    autonManager.addOption("Do Nothing", new InstantCommand());
    autonManager.addOption("Taxi Only", new TaxiOnlyAuton(drivetrainSubsystem));
    autonManager.addDefaultOption(
        "1 and 1",
        new OneAndOneAuton(
            drivetrainSubsystem,
            towerSubsystem,
            turretSubsystem,
            shooterSubsystem,
            limelightSubsystem));
    autonManager.addOption(
        "2 Ball Left Side",
        new TwoBallLeftSide(
            drivetrainSubsystem,
            towerSubsystem,
            turretSubsystem,
            shooterSubsystem,
            limelightSubsystem));
    autonManager.addOption(
        "2 Ball Right Side",
        new TwoBallRightSide(
            drivetrainSubsystem,
            towerSubsystem,
            turretSubsystem,
            shooterSubsystem,
            limelightSubsystem));
    autonManager.addOption(
        "4 Ball Right Side",
        new FourBallRightSide(
            drivetrainSubsystem,
            towerSubsystem,
            turretSubsystem,
            shooterSubsystem,
            limelightSubsystem));
  }

  private void configureButtonBindings() {
    // === DRIVER ===
    drivetrainSubsystem.setDefaultCommand(new Drive(driver, drivetrainSubsystem));
    driver.rightBumper.whenHeld(new IntakeAndIngest(towerSubsystem));
    driver.leftBumper.whenHeld(new ReverseTower(towerSubsystem));

    // === OPERATOR ===
    /// === AUTOMAGIC ===
    operator.buttonBack.whenHeld(new AutoAim(turretSubsystem, limelightSubsystem));
    // operator.buttonBack.toggleWhenPressed(new AutoAim(turretSubsystem, limelightSubsystem));
    operator.buttonStart.whenHeld(
        new ShootInterpolated(towerSubsystem, shooterSubsystem, limelightSubsystem));

    /// === MANUAL ===
    operator.leftBumper.whenHeld(new ReverseTower(towerSubsystem));
    climbSubsystem.setDefaultCommand(new DriveClimber(operator, climbSubsystem));

    /// === SHOOTING ===
    operator.buttonA.whenHeld(new ShootRpm(750, towerSubsystem, shooterSubsystem));
    operator.buttonB.whenHeld(new ShootRpm(1600, towerSubsystem, shooterSubsystem)); // 1480
    operator.buttonX.whenHeld(new ShootRpm(1900, towerSubsystem, shooterSubsystem)); // 1540
    operator.buttonY.whenHeld(new ShootRpm(2150, towerSubsystem, shooterSubsystem)); // 1700

    /// === TURRET ===
    turretSubsystem.setDefaultCommand(new DriveTurret(operator, turretSubsystem));
    operator.dPadLeft.whenHeld(new TurnTurretTo(-90.0, turretSubsystem));
    operator.dPadUp.whenHeld(new TurnTurretTo(0.0, turretSubsystem));
    operator.dPadRight.whenHeld(new TurnTurretTo(90.0, turretSubsystem));
    operator.dPadDown.whenHeld(new TurnTurretTo(180.0, turretSubsystem));

    /// === PROGRAMMER TUNING ===
    // operator.buttonA.whenHeld(new ShootPercent(0.5, shooterSubsystem));
    // operator.buttonA.whenHeld(
    //     new ShootRpmSD(towerSubsystem, shooterSubsystem, colorSensorsSubsystem));
  }

  public Command getAutonomousCommand() {
    return autonManager.getSelected();
  }

  public void turnOffLimelightLEDs() {
    limelightSubsystem.enableDrivingMode();
  }

  public void updateAlliance() {
    towerSubsystem.updateAlliance();
  }
}
