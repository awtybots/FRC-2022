package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

    private final double kBlinkPeriod = .4;

    private final Timer timer = new Timer();
    private State state = State.kOn;

    private final PowerDistribution pdh;

    private enum State {
        kOn,
        kOff,
        kBlinkingOn,
        kBlinkingOff;
    }

    public LedSubsystem() {
        pdh = new PowerDistribution();

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
        pdh.setSwitchableChannel(on);
    }

    // PUBLIC

    public void blink() {
        if (state == State.kBlinkingOn || state == State.kBlinkingOff) return;
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
