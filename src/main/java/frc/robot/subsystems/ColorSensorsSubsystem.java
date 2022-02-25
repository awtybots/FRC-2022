package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorSensors;
import frc.robot.Constants.Field;

public class ColorSensorsSubsystem extends SubsystemBase {
  private final ColorSensor lowerSensor = new ColorSensor(ColorSensors.kLowerSensorPort);
  private final ColorSensor upperSensor = new ColorSensor(ColorSensors.kUpperSensorPort);

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
    // TODO smartdashboard two balls
  }

  private class ColorSensor {
    private final double minimumConfidence = 0.8; // TODO

    private final ColorSensorV3 sensor;
    private final ColorMatch colorMatch;

    public ColorSensor(I2C.Port port) {
      sensor = new ColorSensorV3(port);

      colorMatch = new ColorMatch();
      for (Color color : Field.kBallColors.values()) {
        colorMatch.addColorMatch(color);
        colorMatch.addColorMatch(color);
      }
      colorMatch.setConfidenceThreshold(minimumConfidence);
    }

    public Alliance getDetectedBall() {
      Color detectedColor = sensor.getColor(); // TODO smartdashboard
      ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);

      double proximity = sensor.getProximity(); // 0 to 2047, higher means closer
      // TODO use proximity

      for (Alliance alliance : Field.kBallColors.keySet()) {
        if (match.color == Field.kBallColors.get(alliance)) {
          return alliance;
        }
      }

      return Alliance.Invalid;
    }
  }
}
