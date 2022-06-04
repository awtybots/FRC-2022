// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.util.Controller;

public class DriveClimber extends CommandBase {

    private final ClimbSubsystem climber;
    private final Controller controller;

    public DriveClimber(Controller controller, ClimbSubsystem climbSubsystem) {
        addRequirements(climbSubsystem);
        this.climber = climbSubsystem;
        this.controller = controller;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double forward = controller.rightBumper.get() ? 1 : 0;
        double reverse = controller.getRightTrigger();
        double rate = forward - reverse;
        climber.drive(rate);
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
