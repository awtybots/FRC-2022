package frc.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class Controller {
  private XboxController controller;
  private double kDeadzoneStick = 0.08;
  private double kDeadzoneTrigger = 0.05;

  public JoystickButton buttonA, buttonX, buttonY, buttonB;
  public JoystickButton buttonBack, buttonStart;
  public JoystickButton bumperLeft, bumperRight;

  public POVButton dpadUp, dpadRight, dpadDown, dpadLeft;

  public Button triggerLeft, triggerRight;
  public JoystickButton joystickClickLeft, joystickClickRight;

  public Controller(int port) {
    controller = new XboxController(port);

    buttonA = createButton(XboxController.Button.kA.value);
    buttonX = createButton(XboxController.Button.kX.value);
    buttonY = createButton(XboxController.Button.kY.value);
    buttonB = createButton(XboxController.Button.kB.value);
    buttonBack = createButton(XboxController.Button.kBack.value);
    buttonStart = createButton(XboxController.Button.kStart.value);

    bumperLeft = createButton(XboxController.Button.kLeftBumper.value);
    bumperRight = createButton(XboxController.Button.kRightBumper.value);

    joystickClickLeft = createButton(XboxController.Button.kLeftStick.value);
    joystickClickRight = createButton(XboxController.Button.kRightStick.value);

    dpadUp = new POVButton(controller, 0);
    dpadRight = new POVButton(controller, 90);
    dpadDown = new POVButton(controller, 180);
    dpadLeft = new POVButton(controller, 270);

    triggerLeft = new Button(() -> getLeftTrigger() > 0);
    triggerRight = new Button(() -> getRightTrigger() > 0);
  }

  public double getLeftTrigger() {
    return deadzone(controller.getLeftTriggerAxis(), kDeadzoneTrigger);
  }

  public double getLeftX() {
    return deadzone(controller.getLeftX(), kDeadzoneStick);
  }

  public double getLeftY() {
    return deadzone(-controller.getLeftY(), kDeadzoneStick);
  }

  public double getRightTrigger() {
    return deadzone(controller.getRightTriggerAxis(), kDeadzoneTrigger);
  }

  public double getRightX() {
    return deadzone(controller.getRightX(), kDeadzoneStick);
  }

  public double getRightY() {
    return deadzone(-controller.getRightY(), kDeadzoneStick);
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
