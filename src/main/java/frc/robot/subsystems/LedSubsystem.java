package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

  private final double kBlinkPeriod = 0.5;
  private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  private final Timer timer = new Timer();
  private State state = State.kOn;
  private boolean blinkOn = false;

  private enum State {
    kBlinking,
    kOn,
    kOff,
  }

  public LedSubsystem() {
    turnOn();
  }

  @Override
  public void periodic() {
    if (state != State.kBlinking) return;

    if (timer.get() > kBlinkPeriod) {
      blinkOn = !blinkOn;
      toggle(blinkOn);
      timer.reset();
    }
  }

  private void toggle(boolean on) {
    pdh.setSwitchableChannel(on);
  }

  // PUBLIC

  public void blink() {
    if (state != State.kBlinking) {
      state = State.kBlinking;
      blinkOn = false;
      timer.reset();
      timer.start();
      toggle(false);
    }
  }

  public void turnOn() {
    if (state != State.kOn) {
      state = State.kOn;
      toggle(true);
    }
  }

  public void turnOff() {
    if (state != State.kOff) {
      state = State.kOff;
      toggle(false);
    }
  }
}
