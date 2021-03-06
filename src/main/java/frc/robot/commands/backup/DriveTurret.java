// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.util.Controller;

public class DriveTurret extends CommandBase {

    private final TurretSubsystem turret;
    private final Controller controller;

    public DriveTurret(Controller controller, TurretSubsystem turret) {
        addRequirements(turret);
        this.turret = turret;
        this.controller = controller;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double rate = -controller.getLeftStickX();
        turret.drive(rate);
    }

    @Override
    public void end(boolean interrupted) {
        turret.idle();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
