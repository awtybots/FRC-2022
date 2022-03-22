// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  private final Controller Driver = new Controller(0);
  private final Controller Operator = new Controller(1);

  public static final PowerDistribution pdh = new PowerDistribution(0, ModuleType.kRev);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final TowerSubsystem towerSubsystem = new TowerSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final ColorSensorsSubsystem colorSensorsSubsystem = new ColorSensorsSubsystem();
  public static final LedSubsystem ledSubsystem = new LedSubsystem();

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
    autonChooser.setDefaultOption(
        "1 Low Goal 1 High Goal",
        new TwoBallLowAndHighAuton(
            drivetrainSubsystem,
            intakeSubsystem,
            towerSubsystem,
            turretSubsystem,
            shooterSubsystem,
            limelightSubsystem,
            colorSensorsSubsystem));
    autonChooser.addOption(
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
    drivetrainSubsystem.setDefaultCommand(new Drive(Driver, drivetrainSubsystem));
    Driver.RightBumper.whenHeld(
        new IntakeAndIngest(intakeSubsystem, towerSubsystem, colorSensorsSubsystem));
    Driver.LeftBumper.whenHeld(
        new ReverseIntake(intakeSubsystem).alongWith(new ReverseTower(towerSubsystem)));

    // === OPERATOR ===
    /// === AUTOMAGIC ===
    Operator.ButtonBack.whenHeld(new AutoAim(turretSubsystem, limelightSubsystem));
    Operator.ButtonStart.whenHeld(
        new ShootInterpolated(
            towerSubsystem, shooterSubsystem, limelightSubsystem, colorSensorsSubsystem));

    /// === MANUAL ===
    Operator.LeftBumper.whenHeld(new ReverseTower(towerSubsystem));
    climbSubsystem.setDefaultCommand(new DriveClimber(Operator, climbSubsystem));

    /// === SHOOTING ===
    Operator.ButtonA.whenHeld(
        new ShootRpm(750, towerSubsystem, shooterSubsystem, colorSensorsSubsystem));
    Operator.ButtonB.whenHeld(
        new ShootRpm(1600, towerSubsystem, shooterSubsystem, colorSensorsSubsystem)); // 1480
    Operator.ButtonX.whenHeld(
        new ShootRpm(1800, towerSubsystem, shooterSubsystem, colorSensorsSubsystem)); // 1540
    Operator.ButtonY.whenHeld(
        new ShootRpm(2300, towerSubsystem, shooterSubsystem, colorSensorsSubsystem)); // 1700

    /// === TURRET ===
    turretSubsystem.setDefaultCommand(new DriveTurret(Operator, turretSubsystem));
    Operator.DPadLeft.whenHeld(new TurnTurretTo(-90.0, turretSubsystem));
    Operator.DPadUp.whenHeld(new TurnTurretTo(0.0, turretSubsystem));
    Operator.DPadRight.whenHeld(new TurnTurretTo(90.0, turretSubsystem));
    Operator.DPadDown.whenHeld(new TurnTurretTo(180.0, turretSubsystem));

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

  /**
   * This is kindof hacky but `getAlliance` can return Invalid when not connected to the DS or FMS
   * so it's not safe to assume that upon construction of subsystems we will be connected to the DS
   * or FMS, but by the start of autonomous we will be connected. So that's why this is here. It
   * doesn't really matter during testing because DS will set it and DS will most likely be open
   * before we connect, but it's still possible it won't be able to read a valid Alliance, so just
   * to be sure this is here.
   */
  public void setAlliance(Alliance alliance) {
    colorSensorsSubsystem.setAlliance(alliance);
  }
}
