package frc.robot.util.math;

public final class Convert {

  public static enum Encoder {
    VersaPlanetaryIntegrated(1024.0),
    TalonFXIntegrated(2048.0),
    CTREMagEncoder(4096.0);

    public final double ticksPerRevolution;

    Encoder(double ticksPerRevolution) {
      this.ticksPerRevolution = ticksPerRevolution;
    }
  }

  /** @return encoder ticks per 100ms */
  public static final double degPerSecToEncoderVel(double degPerSec, Encoder encoder) {
    return (degPerSec / 360.0) * (encoder.ticksPerRevolution / 10.0);
  }

  /** @return (encoder ticks per 100ms) per second */
  public static final double degPerSecPerSecToEncoderAccel(double degPerSecPerSec, Encoder encoder) {
    return (degPerSecPerSec / 360.0) * (encoder.ticksPerRevolution / 10.0);
  }

  /** @return encoder ticks per 100ms */
  public static final double rpmToEncoderVel(double rpm, Encoder encoder) {
    return (rpm / 60.0) * (encoder.ticksPerRevolution / 10.0);
  }

  /** @return rpm */
  public static final double encoderVelToRpm(double encoderVel, Encoder encoder) {
    return encoderVel * (10.0 / encoder.ticksPerRevolution) * 60.0;
  }
}
