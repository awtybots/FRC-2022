package frc.robot.commands.testing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import frc.robot.commands.backup.ShootRpmAndFeed;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class ShootRpmSD extends ProxyScheduleCommand {

  public ShootRpmSD(TowerSubsystem tower, ShooterSubsystem shooter) {
    super(new ShootRpmAndFeed(SmartDashboard.getNumber("SH - set rpm", 0), tower, shooter));
    SmartDashboard.putNumber("SH - set rpm", 0);
  }
}
