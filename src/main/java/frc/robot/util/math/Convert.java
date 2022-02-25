// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  public static final double degPerSecAccelToEncoder(double degPerSecPerSec, Encoder encoder) {
    return (degPerSecPerSec / 360.0) * (encoder.ticksPerRevolution / 10.0);
  }
}
