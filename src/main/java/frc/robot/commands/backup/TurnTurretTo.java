package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurnTurretTo extends CommandBase {

    private final TurretSubsystem turret;
    private final double angle;

    public TurnTurretTo(double angle, TurretSubsystem turretSubsystem) {
        addRequirements(turretSubsystem);
        this.turret = turretSubsystem;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        turret.turnTo(angle);
    }

    @Override
    public boolean isFinished() {
        return turret.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        turret.idle();
    }
}
