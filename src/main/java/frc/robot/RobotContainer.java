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

    private final Controller Driver = new Controller(0);
    private final Controller Operator = new Controller(1);

    private final DrivetrainSubsystem Drivetrain = new DrivetrainSubsystem();
    private final TowerSubsystem Tower = new TowerSubsystem();
    private final TurretSubsystem Turret = new TurretSubsystem();
    private final ShooterSubsystem Shooter = new ShooterSubsystem();
    private final ClimbSubsystem Climber = new ClimbSubsystem();

    public static final LimelightSubsystem Limelight = new LimelightSubsystem();
    public static final LedSubsystem LEDs = new LedSubsystem();

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
        autonManager.addOption("1 and 1", new OneAndOneAuton(Drivetrain, Tower, Turret, Shooter));
        autonManager.addOption(
                "2 Ball Left Side", new TwoBallLeftSide(Drivetrain, Tower, Turret, Shooter));
        autonManager.addDefaultOption(
                "2 Ball Right Side", new TwoBallRightSide(Drivetrain, Tower, Turret, Shooter));
        autonManager.addOption(
                "4 Ball Right Side", new FourBallRightSide(Drivetrain, Tower, Turret, Shooter));
    }

    private void configureButtonBindings() {
        // === DRIVER ===
        Drivetrain.setDefaultCommand(new Drive(Driver, Drivetrain));
        Driver.rightBumper.whenHeld(new IntakeAndIngest(Tower));
        Driver.leftBumper.whenHeld(new ReverseTower(Tower));

        // === DRIVER AS OPERATOR ===
        // driver.buttonBack.toggleWhenPressed(new AutoAim(turretSubsystem));
        // driver.buttonStart.whenHeld(
        //     new ShootInterpolated(Tower, Shooter));
        // driver.buttonA.whenHeld(new ShootRpm(750,  Tower, Shooter));
        // driver.buttonB.whenHeld(new ShootRpm(1600, Tower, Shooter));
        // driver.buttonX.whenHeld(new ShootRpm(1900, Tower, Shooter));
        // driver.buttonY.whenHeld(new ShootRpm(2150, Tower, Shooter));

        // === OPERATOR ===
        /// === AUTOMAGIC ===
        // operator.buttonBack.whenHeld(new AutoAim(turretSubsystem));
        Operator.buttonBack.toggleWhenPressed(new AutoAim(Turret));
        Operator.buttonStart.whenHeld(new ShootInterpolated(Tower, Shooter));

        /// === MANUAL ===
        Climber.setDefaultCommand(new DriveClimber(Operator, Climber));
        Operator.leftTrigger.whenHeld(new ReverseTower(Tower));
        Operator.leftBumper.whenPressed(new ActuateTraversalPistons(Climber));
        // ! TODO feed shooter
        // operator.rightTrigger.whenHeld(feed shooter...)

        /// === SHOOTING ===
        Operator.buttonA.whenHeld(new ShootRpm(750, Tower, Shooter));
        Operator.buttonB.whenHeld(new ShootRpm(1600, Tower, Shooter)); // 1480
        Operator.buttonX.whenHeld(new ShootRpm(1900, Tower, Shooter)); // 1540
        Operator.buttonY.whenHeld(new ShootRpm(2150, Tower, Shooter)); // 1700

        /// === TURRET ===
        Turret.setDefaultCommand(new DriveTurret(Operator, Turret));
        Operator.dPadLeft.whenHeld(new TurnTurretTo(-90.0, Turret));
        Operator.dPadUp.whenHeld(new TurnTurretTo(0.0, Turret));
        Operator.dPadRight.whenHeld(new TurnTurretTo(90.0, Turret));
        Operator.dPadDown.whenHeld(new TurnTurretTo(180.0, Turret));

        /// === PROGRAMMER TUNING ===
        // operator.rightBumper.whenHeld(new ShootPercent(0.5, Shooter));
        // operator.rightBumper.whenHeld(new ShootRpmSD(Tower, Shooter));
    }

    public Command getAutonomousCommand() {
        // return new TwoBallLeftSide(Drivetrain, Tower, Turret, Shooter);
        return autonManager.getSelected();
    }

    public void toggleLimelightLeds(boolean on) {
        if (on) {
            Limelight.enableShootingMode();
        } else {
            Limelight.enableDrivingMode();
        }
    }

    public void updateAlliance() {
        Tower.updateAlliance();
    }
}
