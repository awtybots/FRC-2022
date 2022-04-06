package frc.util;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor {

  private final int minimumDistance;
  private final double minColorConfidence;

  private final Color redColor, blueColor;

  private final ColorSensorV3 sensor;
  private final ColorMatch colorMatch = new ColorMatch();
  private final String name;

  public ColorSensor(
      String name, I2C.Port port, Color red, Color blue, int minDistance, double minConfidence) {
    this.minimumDistance = minDistance;
    this.minColorConfidence = minConfidence;
    this.redColor = red;
    this.blueColor = blue;
    this.name = name;

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
  }

  public boolean ballPresent() {
    SmartDashboard.putNumber("PROXIMITY " + this.name, sensor.getProximity());
    return sensor.getProximity() > minimumDistance;
  }

  public Alliance getBallAlliance() {
    Color detectedColor = sensor.getColor();
    ColorMatchResult match = colorMatch.matchColor(detectedColor);

    if (match != null) {
      if (match.color == redColor) return Alliance.Red;
      if (match.color == blueColor) return Alliance.Blue;
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
