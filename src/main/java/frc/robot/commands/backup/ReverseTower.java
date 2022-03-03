package frc.robot.commands.backup;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.TowerSubsystem;

public class ReverseTower extends StartEndCommand {

  public ReverseTower(TowerSubsystem towerSubsystem) {
    super(towerSubsystem::reverseBoth, towerSubsystem::stop, towerSubsystem);
  }
  
}
