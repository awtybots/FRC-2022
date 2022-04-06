// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auton.blind.*;
import frc.robot.commands.backup.*;
import frc.robot.commands.main.*;
import frc.robot.commands.testing.*;
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

  private final Controller Driver = new Controller(0);
  private final Controller Operator = new Controller(1);

  public static final LimelightSubsystem Limelight = new LimelightSubsystem();
  public static final LedSubsystem LEDs = new LedSubsystem();

  private final DrivetrainSubsystem Drivetrain = new DrivetrainSubsystem();
  private final TowerSubsystem Tower = new TowerSubsystem();
  private final TurretSubsystem Turret = new TurretSubsystem();
  private final ShooterSubsystem Shooter = new ShooterSubsystem();
  private final ClimbSubsystem Climber = new ClimbSubsystem();

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
    autonManager.addOption("Taxi Only", new TaxiOnlyAuton(Drivetrain));
    autonManager.addDefaultOption("1 and 1", new OneAndOneAuton(Drivetrain, Tower, Shooter));
    autonManager.addOption("2 Ball Left Side", new TwoBallLeftSide(Drivetrain, Tower, Shooter));
    autonManager.addOption(
        "2 Ball Right Side", new TwoBallRightSide(Drivetrain, Tower, Turret, Shooter));
    autonManager.addOption(
        "4 Ball Right Side", new FourBallRightSide(Drivetrain, Tower, Turret, Shooter));
  }

  private void configureButtonBindings() {

    // === DRIVER ===
    Drivetrain.setDefaultCommand(new Drive(Driver, Drivetrain));
    Driver.RightBumper.whenHeld(new IntakeAndIngest(Tower));
    Driver.LeftBumper.whenHeld(new ReverseTower(Tower));

    // === OPERATOR ===
    /// === AUTOMAGIC ===
    Operator.ButtonBack.toggleWhenPressed(new AutoAim(Turret));
    Operator.RightBumper.whenHeld(new SpinupInterpolated(Shooter));

    /// === MANUAL ===
    Climber.setDefaultCommand(new DriveClimber(Operator, Climber));
    Operator.LeftBumper.whenHeld(new ReverseTower(Tower));
    // Operator.LeftTrigger.whenHeld(new SeparateBalls(Tower));

    /// === SHOOTING ===
    // Operator.RightTrigger.whenHeld(new FeedShooter(Tower, Shooter, Turret));
    Operator.ButtonA.whenHeld(new SpinupRpmAndFeed(750, Tower, Shooter));
    Operator.ButtonB.whenHeld(new SpinupRpmAndFeed(1600, Tower, Shooter)); // 1480
    Operator.ButtonX.whenHeld(new SpinupRpmAndFeed(1900, Tower, Shooter)); // 1540
    Operator.ButtonY.whenHeld(new SpinupRpmAndFeed(2150, Tower, Shooter)); // 1700

    /// === TURRET ===
    Turret.setDefaultCommand(new DriveTurret(Operator, Turret));
    Operator.DPadLeft.whenHeld(new TurnTurretTo(-90.0, Turret));
    Operator.DPadUp.whenHeld(new TurnTurretTo(0.0, Turret));
    Operator.DPadRight.whenHeld(new TurnTurretTo(90.0, Turret));
    Operator.DPadDown.whenHeld(new TurnTurretTo(180.0, Turret));

    /// === PROGRAMMER TUNING ===
    if (Constants.TUNING_MODE) {
      SmartDashboard.putNumber("SH - set rpm", 0);
      Operator.LeftTrigger.whenHeld(new ShootRpmSD(Tower, Shooter));
    }
  }

  public Command getAutonomousCommand() {
    return autonManager.getSelected();
  }

  public void turnOffLimelightLEDs() {
    Limelight.enableDrivingMode();
  }

  public void updateAlliance() {
    Tower.updateAlliance();
  }
}
