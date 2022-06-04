package frc.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.HashMap;

public class AutonManager {
    private HashMap<String, Command> autonSequences;
    private SendableChooser<String> dashboardSelector = new SendableChooser<String>();

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
            dashboardSelector.setDefaultOption(name, name);
        } else {
            dashboardSelector.addOption(name, name);
        }
    }

    public void addOption(String name, Command seq) {
        addOption(name, seq, false);
    }

    public void addDefaultOption(String name, Command seq) {
        addOption(name, seq, true);
    }

    public Command getSelected() {
        String dashboardSelection =
                NetworkTableInstance.getDefault()
                        .getTable("SmartDashboard")
                        .getSubTable(autonSelectorKey)
                        .getEntry("selected")
                        .getString("_");

        if (dashboardSelection == "_") {
            System.out.printf(
                    "No auton retrieved from NetworkTablesEntry `SmartDashboard/%s/selected`\n",
                    autonSelectorKey);
            if (defaultCommand == null) {
                System.out.println("Doing nothing for autonomous.");
                return new InstantCommand();
            } else {
                System.out.printf("Running default autonomous: `%s`", defaultCommand.getName());
                return defaultCommand;
            }
        } else {
            final Command autonCommand = autonSequences.get(dashboardSelection);

            System.out.printf(
                    "Retrieved auton selection: `%s` from `SmartDashboard/%s/selected`\n",
                    dashboardSelection, autonSelectorKey);

            if (autonCommand != null) {
                System.out.printf("Running `%s` for autonomous\n", autonCommand.getName());
                return autonSequences.get(dashboardSelection);
            } else {
                System.out.printf("Auton selection of `%s` was not found\n", dashboardSelection);
                // If it ever reaches this branch of the code, it is most likely the fault of
                // something within this file. Either that or NetworkTables somehow sent us an
                // invalid selection back; however, that seems highly unlikely.
                if (defaultCommand != null) {
                    System.out.printf(
                            "Running default autonomous `%s`\n", defaultCommand.getName());
                    return defaultCommand;
                } else {
                    System.out.println(
                            "No default autonomous was selected. Doing nothing for autonomous.");
                    return new InstantCommand();
                }
            }
        }
    }

    public void displayChoices() {
        SmartDashboard.putData(autonSelectorKey, dashboardSelector);
    }
}
