package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootRpmOrSpit extends CommandBase {
    private final TowerSubsystem tower;
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;

    private final double rpm;

    public ShootRpmOrSpit(
            double rpm,
            TowerSubsystem towerSubsystem,
            ShooterSubsystem shooterSubsystem,
            TurretSubsystem turretSubsystem) {

        this.tower = towerSubsystem;
        this.shooter = shooterSubsystem;
        this.turret = turretSubsystem;

        this.rpm = rpm;

        addRequirements(towerSubsystem, shooterSubsystem, turretSubsystem);
    }

    @Override
    public void initialize() {}

    private void executeShoot() {
        turret.trackTarget(
                RobotContainer.Limelight.hasVisibleTarget(),
                RobotContainer.Limelight.angleToTarget());

        shooter.shootRpm(this.rpm);
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
        shooter.stop();
        tower.stop();
        turret.idle();
    }
}
