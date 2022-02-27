package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ColorSensors;
import frc.robot.Constants.Field;

public class ColorSensorsSubsystem extends SubsystemBase {
  private final ColorSensor lowerSensor = new ColorSensor(ColorSensors.kLowerSensorPort, "lower");
  private final ColorSensor upperSensor = new ColorSensor(ColorSensors.kUpperSensorPort, "upper");

  private Alliance lowerBall = Alliance.Invalid;
  private Alliance upperBall = Alliance.Invalid;

  public boolean isLowerBallPresent() {
    return lowerBall != Alliance.Invalid;
  }

  public boolean isUpperBallPresent() {
    return upperBall != Alliance.Invalid;
  }

  public boolean isLowerBallOurs() {
    return lowerBall == DriverStation.getAlliance();
  }

  public boolean isUpperBallOurs() {
    return upperBall == DriverStation.getAlliance();
  }

  @Override
  public void periodic() {
    lowerBall = lowerSensor.getDetectedBall();
    upperBall = upperSensor.getDetectedBall();

    SmartDashboard.putString("TW - lower ball", lowerBall.toString());
    SmartDashboard.putString("TW - upper ball", upperSensor.toString());
  }

  private class ColorSensor {
    // FIXME set correct confidence threshold for color sensors
    private final double minimumConfidence = 0.8;

    private final ColorSensorV3 sensor;
    private final ColorMatch colorMatch;

    private final String id;

    public ColorSensor(I2C.Port port, String id) {
      this.id = id;

      sensor = new ColorSensorV3(port);

      colorMatch = new ColorMatch();
      for (Color color : Field.kBallColors.values()) {
        colorMatch.addColorMatch(color);
        colorMatch.addColorMatch(color);
      }
      colorMatch.setConfidenceThreshold(minimumConfidence);
    }

    public Alliance getDetectedBall() {
      Color detectedColor = sensor.getColor();
      ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);

      // TODO use proximity value from sensor for better detection accuracy
      double proximity = sensor.getProximity(); // 0 to 2047, higher means closer

      if (Constants.TUNING_MODE) {
        SmartDashboard.putString("TW - " + id + " raw color", detectedColor.toString());
        SmartDashboard.putNumber("TW - " + id + " confidence", match.confidence);
        SmartDashboard.putNumber("TW - " + id + " proximity", proximity);
      }

      for (Alliance alliance : Field.kBallColors.keySet()) {
        if (match.color == Field.kBallColors.get(alliance)) {
          return alliance;
        }
      }

      return Alliance.Invalid;
    }
  }
}
