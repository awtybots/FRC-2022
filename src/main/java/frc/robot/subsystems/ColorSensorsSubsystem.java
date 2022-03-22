package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ColorSensors;

public class ColorSensorsSubsystem extends SubsystemBase {

  private final ColorSensor lowerSensor;
  private final ColorSensor upperSensor;

  private final Alliance ourAlliance = DriverStation.getAlliance();

  private Alliance lowerBall = Alliance.Invalid;
  private Alliance upperBall = Alliance.Invalid;

  private static final int kMinProximity = 250;

  public ColorSensorsSubsystem() {
    lowerSensor = new ColorSensor(ColorSensors.kLowerSensorPort);
    upperSensor = new ColorSensor(ColorSensors.kUpperSensorPort);
  }

  public boolean lowerBallOurs() {
    return lowerBall == ourAlliance;
  }

  public boolean upperBallOurs() {
    return upperBall == ourAlliance;
  }

  public boolean lowerBallPresent() {
    return lowerSensor.ballPresent();
  }

  public boolean upperBallPresent() {
    return upperSensor.ballPresent();
  }

  public String upperBallAlliance() {
    return upperBall.toString();
  }

  public String lowerBallAlliance() {
    return lowerBall.toString();
  }

  @Override
  public void periodic() {
    lowerBall = lowerSensor.getDetectedBall();
    upperBall = upperSensor.getDetectedBall();

    if (Constants.TUNING_MODE) {
      SmartDashboard.putString("Upper Ball Color", upperBallAlliance());
      SmartDashboard.putString("Lower Ball Color", lowerBallAlliance());
      SmartDashboard.putString("Upper Ball RGB", upperSensor.getColor());
      SmartDashboard.putString("Lower Ball RGB", lowerSensor.getColor());
    }
  }

  private class ColorSensor {

    private final double minimumConfidence = 0.90;
    private final int minimumDistance = 800;

    // These are currently in RGB but converting the RGB values from the
    // sensor to HSV then matching the colors might be more robust
    // * Tune these at the field if you want to know the color of the balls
    private final Color Red = new Color(0.18, 0.41, 0.43);
    private final Color Blue = new Color(0.45, 0.39, 0.16);

    private final ColorSensorV3 sensor;
    private final ColorMatch colorMatch;

    public ColorSensor(I2C.Port port) {
      sensor = new ColorSensorV3(port);
      sensor.configureColorSensor(
          ColorSensorResolution.kColorSensorRes17bit,
          ColorSensorMeasurementRate.kColorRate25ms,
          GainFactor.kGain3x);
      sensor.configureProximitySensor(
          ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate6ms);

      colorMatch = new ColorMatch();
      colorMatch.addColorMatch(Red);
      colorMatch.addColorMatch(Blue);
      colorMatch.setConfidenceThreshold(minimumConfidence);
    }

    public boolean ballPresent() {
      return sensor.getProximity() > minimumDistance;
    }

    public Alliance getDetectedBall() {
      Color detectedColor = sensor.getColor();
      ColorMatchResult match = colorMatch.matchColor(detectedColor);

      if (match != null) {
        if (match.color == Red) return Alliance.Red;
        if (match.color == Blue) return Alliance.Blue;
      }

      return Alliance.Invalid;
    }

    public String getColor() {
      return colorToString(sensor.getColor());
    }

    private String colorToString(Color c) {
      return String.format("rgb(%.2f, %.2f, %.2f)", c.red, c.green, c.blue);
    }
  }
}
