// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean DEBUG_MODE = true;

  public static final class Drivetrain {
    public static final int kLeftFront = 0;
    public static final int kLeftBack = 1;
    public static final int kRightFront = 2;
    public static final int kRightBack = 3;

    public static final TalonFXConfiguration motorConfig() {
      TalonFXConfiguration conf = new TalonFXConfiguration();
      conf.voltageCompSaturation = 12.0;
      conf.openloopRamp = 1.0;
      return conf;
    }
  }
}
