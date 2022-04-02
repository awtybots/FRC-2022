package frc.util;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public abstract class AutonManager {
  private HashMap<String, Command> autonSequences;
    private SendableChooser<String> autonSelector = new SendableChooser<String>();

    private Command currentSequence;

    private final String autonSelectorKey = "AutonChooser";

    public AutonManager() {
        autonSequences = new HashMap<String, Command>();

        addSequence("Do Nothing", new InstantCommand());

        init();
        displayChoices();
    }

    protected abstract void init(); // this is where we add sequences

    protected void addSequence(String name, Command seq) {
        autonSequences.put(name, seq);

        if (autonSequences.size() == 0) {
            autonSelector.setDefaultOption(name, name);
        } else {
            autonSelector.addOption(name, name);
        }
    }

    public void runCurrentSequence() {
      if(currentSequence == null) currentSequence = new InstantCommand();
      currentSequence.schedule();
    }

    public void chooseCurrentSequence() {
        String autoChoice = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(autonSelectorKey)
                .getEntry("selected").getString("N/A");

        if (autonSequences.containsKey(autoChoice)) {
            System.out.println("Switching to auto: " + autoChoice);
            currentSequence = autonSequences.get(autoChoice);
        }
    }

    /**
     * Set sequence with sequence object
     */
    public void setCurrentSequence(Command seq) {
        currentSequence = seq;
        // TODO run it
    }

    /**
     * Send sequence list to SmartDashboard
     */
    public void displayChoices() {
        SmartDashboard.putData(autonSelectorKey, autonSelector);
    }
}
