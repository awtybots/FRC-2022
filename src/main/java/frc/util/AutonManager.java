package frc.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.HashMap;

public class AutonManager {
  private HashMap<String, Command> autonSequences;
  private SendableChooser<String> autonSelector = new SendableChooser<String>();

  private Command defaultCommand = null;

  private final String autonSelectorKey = "AutonChooser";

  public AutonManager() {
    autonSequences = new HashMap<String, Command>();
    defaultCommand = null;
  }

  private void addOption(String name, Command seq, boolean isDefault) {
    autonSequences.put(name, seq);

    if (isDefault) {
      defaultCommand = seq;
      autonSelector.setDefaultOption(name, name);
    } else {
      autonSelector.addOption(name, name);
    }
  }

  public void addOption(String name, Command seq) {
    addOption(name, seq, false);
  }

  public void addDefaultOption(String name, Command seq) {
    addOption(name, seq, true);
  }

  public Command getSelected() {
    String autoChoice =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getSubTable(autonSelectorKey)
            .getEntry("selected")
            .getString("_");

    if (autoChoice == "_") {
      System.out.println(
          "No auton retrieved from NetworkTablesEntry `SmartDashboard/"
              + autonSelectorKey
              + "/selected`");
      if (defaultCommand == null) {
        System.out.println("Doing nothing for autonomous.");
        return new InstantCommand();
      } else {
        System.out.println("Runnning `" + defaultCommand.getName() + "` for autonomous");
        return defaultCommand;
      }
    }

    if (autonSequences.containsKey(autoChoice)) {
      return autonSequences.get(autoChoice);
    } else {
      if (defaultCommand != null) {
        return defaultCommand;
      } else {
        return new InstantCommand();
      }
    }
  }

  public void displayChoices() {
    SmartDashboard.putData(autonSelectorKey, autonSelector);
  }
}
