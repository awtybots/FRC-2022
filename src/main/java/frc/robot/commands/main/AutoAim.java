package frc.robot.commands.main;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAim extends CommandBase {
    private final TurretSubsystem turret;

    private static final String sdKey = "AutoAim Status";

    public AutoAim(TurretSubsystem turretSubsystem) {
        this.turret = turretSubsystem;

        addRequirements(turretSubsystem);

        SmartDashboard.putBoolean(sdKey, false);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean(sdKey, true);
    }

    @Override
    public void execute() {
        turret.trackTarget(
                RobotContainer.Limelight.hasVisibleTarget(),
                RobotContainer.Limelight.angleToTarget());
    }

    @Override
    public void end(boolean interrupted) {
        turret.idle();

        SmartDashboard.putBoolean(sdKey, false);
    }
}
