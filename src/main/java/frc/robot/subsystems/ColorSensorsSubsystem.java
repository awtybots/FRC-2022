package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ColorSensors;
import frc.util.math.Convert;

public class ColorSensorsSubsystem extends SubsystemBase {

  private final ColorSensor lowerSensor;
  private final ColorSensor upperSensor;

  private Alliance ourAlliance = Alliance.Invalid;

  private Alliance lowerBall = Alliance.Invalid;
  private Alliance upperBall = Alliance.Invalid;

  private static final int kMinProximity = 250;

  public ColorSensorsSubsystem() {
    lowerSensor = new ColorSensor(ColorSensors.kLowerSensorPort);
    upperSensor = new ColorSensor(ColorSensors.kUpperSensorPort);
  }

  public void setAlliance(Alliance alliance) {
    this.ourAlliance = alliance;
  }

  public boolean upperBallOurs() {
    return upperBall == ourAlliance;
  }

  public boolean upperBallPresent() {
    return upperSensor.ballPresent();
  }

  public boolean lowerBallPresent() {
    return lowerSensor.ballPresent();
  }

  @Override
  public void periodic() {
    lowerBall = lowerSensor.getDetectedBall();
    upperBall = upperSensor.getDetectedBall();

    if (Constants.TUNING_MODE) {
      SmartDashboard.putString("Upper Ball Color", upperBall.toString());
      SmartDashboard.putString("Lower Ball Color", lowerBall.toString());
      SmartDashboard.putString("Upper Ball RGB", upperSensor.rawColor());
      SmartDashboard.putString("Lower Ball RGB", lowerSensor.rawColor());
    }
  }

  private class ColorSensor {

    private final int minimumDistance = 800; // TODO find a good value
    private final double minColorConfidence = 0.90; // TODO make sure this a correct value

    // Tune these at each field if you want to know the color of the balls
    private final Color Red = Convert.rgbToHSV(new Color(0.18, 0.41, 0.43));
    private final Color Blue = Convert.rgbToHSV(new Color(0.45, 0.39, 0.16));

    private final ColorSensorV3 sensor;
    private final ColorMatch colorMatch = new ColorMatch();

    public ColorSensor(I2C.Port port) {
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

      colorMatch.addColorMatch(Red);
      colorMatch.addColorMatch(Blue);
      colorMatch.setConfidenceThreshold(minColorConfidence);
    }

    public boolean ballPresent() {
      return sensor.getProximity() > minimumDistance;
    }

    public Alliance getDetectedBall() {
      // Match color in HSV space for _possible_ better robustness
      Color detectedColor = Convert.rgbToHSV(sensor.getColor());
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
}
