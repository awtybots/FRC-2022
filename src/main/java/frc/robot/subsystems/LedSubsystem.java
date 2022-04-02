package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

  private final double kBlinkPeriod = 0.5;

  private final Timer timer = new Timer();
  private State state = State.kOn;

  private enum State {
    kOn,
    kOff,
    kBlinkingOn,
    kBlinkingOff;
  }

  public LedSubsystem() {
    turnOn();
  }

  @Override
  public void periodic() {
    switch (state) {
      case kBlinkingOn:
        if (timer.get() > kBlinkPeriod) {
          state = State.kBlinkingOff;
          timer.reset();
          toggle(false);
        }
        break;
      case kBlinkingOff:
        if (timer.get() > kBlinkPeriod) {
          state = State.kBlinkingOn;
          timer.reset();
          toggle(true);
        }
      default:
        break;
    }
  }

  private void toggle(boolean on) {
    // RobotContainer.pdh.setSwitchableChannel(on);
  }

  // PUBLIC

  public void blink() {
    state = State.kBlinkingOff;
    timer.reset();
    timer.start();
    toggle(false);
  }

  public void turnOn() {
    state = State.kOn;
    toggle(true);
  }

  public void turnOff() {
    state = State.kOff;
    toggle(false);
  }
}
