package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class ShootRpm extends CommandBase {

    private final TowerSubsystem tower;
    private final ShooterSubsystem shooter;
    private final double rpm;

    public ShootRpm(double rpm, TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem) {
        this.tower = towerSubsystem;
        this.shooter = shooterSubsystem;
        this.rpm = rpm;

        addRequirements(towerSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooter.shootRpm(this.rpm);
        RobotContainer.LEDs.blink();
    }

    @Override
    public void execute() {
        tower.feed(shooter.isAtTarget());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        tower.stop();
        RobotContainer.LEDs.turnOn();
    }
}
