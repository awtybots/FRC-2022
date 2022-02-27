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

  public static double rpmToEncoderVel(double rpm, Encoder encoder) {
    return (rpm / 60.0) * (encoder.ticksPerRevolution / 10.0);
  }

  /** @return encoder ticks per 100ms */
  public static double degPerSecToEncoderVel(double degPerSec, Encoder encoder) {
    return (degPerSec / 360.0) * (encoder.ticksPerRevolution / 10.0);
  }

  /** @return (encoder ticks per 100ms) per second */
  public static double degPerSecPerSecToEncoderAccel(double degPerSecPerSec, Encoder encoder) {
    return (degPerSecPerSec / 360.0) * (encoder.ticksPerRevolution / 10.0);
  }

  public static double encoderVelToRpm(double sensorUnitsPer100ms, Encoder encoder) {
    return ((sensorUnitsPer100ms * 10.0) / encoder.ticksPerRevolution) * 60.0;
  }
}
