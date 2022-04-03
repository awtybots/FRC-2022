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

  private final Controller driver = new Controller(0);
  private final Controller operator = new Controller(1);

  // public static final PowerDistribution pdh = new PowerDistribution(0, ModuleType.kRev);

  private final DrivetrainSubsystem Drivetrain = new DrivetrainSubsystem();
  private final TowerSubsystem Tower = new TowerSubsystem();
  private final TurretSubsystem Turret = new TurretSubsystem();
  private final ShooterSubsystem Shooter = new ShooterSubsystem();
  private final ClimbSubsystem Climber = new ClimbSubsystem();
  private final LimelightSubsystem Limelight = new LimelightSubsystem();
  public static final LedSubsystem LEDs = new LedSubsystem();

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
    autonManager.addOption("Taxi Only", new TaxiOnlyAuton(Drivetrain));
    autonManager.addDefaultOption(
        "1 and 1", new OneAndOneAuton(Drivetrain, Tower, Turret, Shooter, Limelight));
    autonManager.addOption(
        "2 Ball Left Side", new TwoBallLeftSide(Drivetrain, Tower, Turret, Shooter, Limelight));
    autonManager.addOption(
        "2 Ball Right Side", new TwoBallRightSide(Drivetrain, Tower, Turret, Shooter, Limelight));
    autonManager.addOption(
        "4 Ball Right Side", new FourBallRightSide(Drivetrain, Tower, Turret, Shooter, Limelight));
  }

  private void configureButtonBindings() {
    // === DRIVER ===
    Drivetrain.setDefaultCommand(new Drive(driver, Drivetrain));
    driver.rightBumper.whenHeld(new IntakeAndIngest(Tower));
    driver.leftBumper.whenHeld(new ReverseTower(Tower));

    // === OPERATOR ===
    /// === AUTOMAGIC ===
    operator.buttonBack.whenHeld(new AutoAim(Turret, Limelight));
    operator.buttonStart.whenHeld(new ShootInterpolated(Shooter, Limelight));

    /// === MANUAL ===
    operator.leftBumper.whenHeld(new ReverseTower(Tower));
    Climber.setDefaultCommand(new DriveClimber(operator, Climber));

    /// === SHOOTING ===
    operator.rightTrigger.whenHeld(new FeedShooter(Tower, Shooter, Turret));
    operator.buttonA.whenHeld(new SpinupRpm(750, Shooter));
    operator.buttonB.whenHeld(new SpinupRpm(1600, Shooter)); // 1480
    operator.buttonX.whenHeld(new SpinupRpm(1900, Shooter)); // 1540
    operator.buttonY.whenHeld(new SpinupRpm(2150, Shooter)); // 1700

    /// === TURRET ===
    Turret.setDefaultCommand(new DriveTurret(operator, Turret));
    operator.dPadLeft.whenHeld(new TurnTurretTo(-90.0, Turret));
    operator.dPadUp.whenHeld(new TurnTurretTo(0.0, Turret));
    operator.dPadRight.whenHeld(new TurnTurretTo(90.0, Turret));
    operator.dPadDown.whenHeld(new TurnTurretTo(180.0, Turret));

    /// === PROGRAMMER TUNING ===
    if (Constants.TUNING_MODE) {
      SmartDashboard.putNumber("SH - set rpm", 0);
      operator.leftTrigger.whenHeld(new ShootRpmSD(Tower, Shooter));
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
