package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooter;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class ShootInterpolatedOrSpit extends CommandBase {

    private final TowerSubsystem tower;
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;

    public ShootInterpolatedOrSpit(
            TowerSubsystem towerSubsystem,
            ShooterSubsystem shooterSubsystem,
            TurretSubsystem turretSubsystem) {

        this.tower = towerSubsystem;
        this.shooter = shooterSubsystem;
        this.turret = turretSubsystem;

        addRequirements(towerSubsystem, shooterSubsystem, turretSubsystem);
    }

    @Override
    public void initialize() {}

    private void executeShoot() {
        if (!RobotContainer.Limelight.hasVisibleTarget()) {
            turret.trackTarget(false);
            shooter.stop();
            return;
        }

        double goalDisplacement = RobotContainer.Limelight.distToTarget();
        if (goalDisplacement == -1) {
            turret.trackTarget(false);
            shooter.stop();
            return;
        }

        double visionTargetXOffset = RobotContainer.Limelight.angleToTarget();
        turret.trackTarget(true, visionTargetXOffset);

        double launchRpm = Shooter.shotMap.calculateShot(goalDisplacement);
        shooter.shootRpm(launchRpm);
    }

    private void executeSpit() {
        turret.spit(
                RobotContainer.Limelight.hasVisibleTarget(),
                RobotContainer.Limelight.angleToTarget());
        shooter.spit();
    }

    @Override
    public void execute() {
        if (tower.upperBallPresent()) {
            if (tower.upperBallOurs()) {
                executeShoot();
            } else {
                executeSpit();
            }

            tower.feed(turret.isAtTarget() && shooter.isAtTarget());
        } else {
            executeShoot(); // idle shooter
            tower.intake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        tower.stop();
        shooter.stop();
        turret.idle();
    }
}
