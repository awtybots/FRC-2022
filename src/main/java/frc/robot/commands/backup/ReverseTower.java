package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.TowerV2Subsystem;

public class ReverseTower extends StartEndCommand {

  public ReverseTower(TowerV2Subsystem towerSubsystem) {
    super(towerSubsystem::reverse, towerSubsystem::stop, towerSubsystem);
  }
}
