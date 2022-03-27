package frc.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.*;

public class Controller {
  private final XboxController controller;
  private final double kDeadzoneStick = 0.08;
  private final double kDeadzoneTrigger = 0.05;

  public final JoystickButton ButtonA, ButtonX, ButtonY, ButtonB;
  public final JoystickButton ButtonBack, ButtonStart;
  public final JoystickButton LeftBumper, RightBumper;

  // D-Pad
  public POVButton DPadUp, DPadRight, DPadDown, DPadLeft;

  /** Allows using the triggers as buttons, for example <pre> mController.leftTrigger.whenPressed(new CommandToRun());*/
  public Button LeftTrigger, RightTrigger;

  /** @param port The port index on the Driver Station that the controller is plugged into. */
  public Controller(int port) {
    controller = new XboxController(port);

    ButtonA = createButton(XboxController.Button.kA.value);
    ButtonX = createButton(XboxController.Button.kX.value);
    ButtonY = createButton(XboxController.Button.kY.value);
    ButtonB = createButton(XboxController.Button.kB.value);
    ButtonBack = createButton(XboxController.Button.kBack.value);
    ButtonStart = createButton(XboxController.Button.kStart.value);

    LeftBumper = createButton(XboxController.Button.kLeftBumper.value);
    RightBumper = createButton(XboxController.Button.kRightBumper.value);

    DPadUp = new POVButton(controller, 0);
    DPadRight = new POVButton(controller, 90);
    DPadDown = new POVButton(controller, 180);
    DPadLeft = new POVButton(controller, 270);

    LeftTrigger = new Button(() -> getLeftTrigger() > kDeadzoneTrigger);
    RightTrigger = new Button(() -> getRightTrigger() > kDeadzoneTrigger);
  }

  /** The X (left/right) position of the right joystick on the controller from -1.0 to 1.0 */
  public double getRightStickX() {
    return deadzone(controller.getRightX(), kDeadzoneStick);
  }

  /** The Y (up/down) position of the right joystick on the controller from -1.0 to 1.0 */
  public double getRightStickY() {
    return deadzone(-controller.getRightY(), kDeadzoneStick);
  }

  /** The X (left/right) position of the left joystick on the controller from -1.0 to 1.0 */
  public double getLeftStickX() {
    return deadzone(controller.getLeftX(), kDeadzoneStick);
  }

  /** The Y (up/down) position of the left joystick on the controller from -1.0 to 1.0 */
  public double getLeftStickY() {
    return deadzone(-controller.getLeftY(), kDeadzoneStick);
  }

  /** How much the left trigger on the controller is pressed from 0.0 to 1.0 */
  public double getLeftTrigger() {
    return deadzone(controller.getLeftTriggerAxis(), kDeadzoneTrigger);
  }

  /** How much the right trigger on the controller is pressed from 0.0 to 1.0 */
  public double getRightTrigger() {
    return deadzone(controller.getRightTriggerAxis(), kDeadzoneTrigger);
  }

  // --- Utilities --- //
  private JoystickButton createButton(int buttonID) {
    return new JoystickButton(this.controller, buttonID);
  }

  private double deadzone(double x, double dz) {
    if (Math.abs(x) > dz) return (x - dz * Math.signum(x)) / (1.0 - dz);
    else return 0.0;
  }
}
