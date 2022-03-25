package frc.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ToggleCommand extends CommandBase {
  private final Command command;
  private final JoystickButton btn;

  private boolean on = false;

  public ToggleCommand(Command command, JoystickButton btn) {
    this.command = command;
    this.btn = btn;
  }

  @Override
  public void initialize() {
    on = false;
  }

  @Override
  public void execute() {
    if (btn.get() == on) return;

    on = btn.get();
    if (on) {
      command.schedule();
    } else {
      command.cancel();
    }
  }

  @Override
  public void end(boolean interrupted) {
    command.cancel();
  }
}
