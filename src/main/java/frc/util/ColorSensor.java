// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor {

  private final int minimumDistance;
  private final double minColorConfidence;

  private final Color Red, Blue;

  private final ColorSensorV3 sensor;
  private final ColorMatch colorMatch = new ColorMatch();

  public ColorSensor(I2C.Port port, Color red, Color blue, int minDistance, double minConfidence) {
    this.minimumDistance = minDistance;
    this.minColorConfidence = minConfidence;
    this.Red = red;
    this.Blue = blue;

    colorMatch.addColorMatch(red);
    colorMatch.addColorMatch(blue);
    colorMatch.setConfidenceThreshold(minColorConfidence);

    sensor = new ColorSensorV3(port);

    sensor.configureColorSensor(
        ColorSensorResolution.kColorSensorRes17bit,
        ColorSensorMeasurementRate.kColorRate25ms,
        GainFactor.kGain3x);
    sensor.configureProximitySensor(
        ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate6ms);

    sensor.configureProximitySensor(
        ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate6ms);

    sensor.configureColorSensor(
        ColorSensorResolution.kColorSensorRes17bit,
        ColorSensorMeasurementRate.kColorRate50ms,
        GainFactor.kGain3x);
  }

  public boolean ballPresent() {
    return sensor.getProximity() > minimumDistance;
  }

  public Alliance getBallAlliance() {
    Color detectedColor = sensor.getColor();
    ColorMatchResult match = colorMatch.matchColor(detectedColor);

    if (match != null) {
      if (match.color == Red) return Alliance.Red;
      if (match.color == Blue) return Alliance.Blue;
    }

    return Alliance.Invalid;
  }

  public String rawColor() {
    return rgbToString(sensor.getColor());
  }

  private String rgbToString(Color c) {
    return String.format("RGB(%.2f, %.2f, %.2f)", c.red, c.green, c.blue);
  }
}
