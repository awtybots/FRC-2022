// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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

  // public static final PowerDistribution pdh = new PowerDistribution(0, ModuleType.kRev);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final TowerV2Subsystem towerV2Subsystem = new TowerV2Subsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  public static final LedSubsystem ledSubsystem = new LedSubsystem();

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    addAutonomousChoices();
    SmartDashboard.putData(autonChooser);
    configureButtonBindings();
    miscellaneousStartup();
  }

  private void miscellaneousStartup() {
    LiveWindow.disableAllTelemetry();
    CameraServer.startAutomaticCapture();
  }

  private void addAutonomousChoices() {
    autonChooser.addOption("Do Nothing", new InstantCommand());
    autonChooser.addOption("Zero Ball Auton", new TaxiOffTarmacAuton(drivetrainSubsystem));
    autonChooser.setDefaultOption(
        "1 Low Goal 1 High Goal",
        new TwoBallLowAndHighAuton(
            drivetrainSubsystem,
            towerV2Subsystem,
            turretSubsystem,
            shooterSubsystem,
            limelightSubsystem));
    autonChooser.addOption(
        "2 Ball High Goal",
        new TwoBallHighGoalAuton(
            drivetrainSubsystem,
            towerV2Subsystem,
            turretSubsystem,
            shooterSubsystem,
            limelightSubsystem));
    // autonChooser.addOption(
    //     "One Ball Auton",
    //     new OneBallAuton(
    //         drivetrainSubsystem, towerSubsystem, shooterSubsystem, colorSensorsSubsystem));
    // autonChooser.addOption(
    //     "2 Ball Stupid",
    //     new TwoBallAutonStupid(
    //         drivetrainSubsystem,
    //         towerSubsystem,
    //         turretSubsystem,
    //         shooterSubsystem,
    //         limelightSubsystem,
    //         colorSensorsSubsystem));
    // autonChooser.addOption(
    //     "Two Ball Auton",
    //     new TwoBallAuton(
    //         drivetrainSubsystem,
    //         towerSubsystem,
    //         turretSubsystem,
    //         shooterSubsystem,
    //         limelightSubsystem,
    //         colorSensorsSubsystem));
    // autonChooser.addOption(
    //     "Four Ball Auton",
    //     new FourBallAuton(
    //         drivetrainSubsystem,
    //         towerSubsystem,
    //         turretSubsystem,
    //         shooterSubsystem,
    //         limelightSubsystem,
    //         colorSensorsSubsystem));
  }

  private void configureButtonBindings() {
    // === DRIVER ===
    drivetrainSubsystem.setDefaultCommand(new Drive(driver, drivetrainSubsystem));
    driver.rightBumper.whenHeld(new IntakeAndIngest(towerV2Subsystem));
    driver.leftBumper.whenHeld(new ReverseTower(towerV2Subsystem));

    // === OPERATOR ===
    /// === AUTOMAGIC ===
    DriveTurret driveTurretCommand = new DriveTurret(operator, turretSubsystem);
    operator.buttonBack.whenPressed(new ToggleAutoAim(driveTurretCommand, turretSubsystem, limelightSubsystem));
    operator.buttonStart.whenHeld(
        new ShootInterpolated(
            towerV2Subsystem, shooterSubsystem, limelightSubsystem));

    /// === MANUAL ===
    operator.leftBumper.whenHeld(new ReverseTower(towerV2Subsystem));
    climbSubsystem.setDefaultCommand(new DriveClimber(operator, climbSubsystem));

    /// === SHOOTING ===
    operator.buttonA.whenHeld(new ShootRpm(750, towerV2Subsystem, shooterSubsystem));
    operator.buttonB.whenHeld(new ShootRpm(1600, towerV2Subsystem, shooterSubsystem)); // 1480
    operator.buttonX.whenHeld(new ShootRpm(1800, towerV2Subsystem, shooterSubsystem)); // 1540
    operator.buttonY.whenHeld(new ShootRpm(2300, towerV2Subsystem, shooterSubsystem)); // 1700

    /// === TURRET ===
    turretSubsystem.setDefaultCommand(driveTurretCommand);
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
    return autonChooser.getSelected();
  }

  public void turnOffLimelightLEDs() {
    limelightSubsystem.drivingMode();
  }

  public void updateAlliance() {
    towerV2Subsystem.updateAlliance();
  }
}
