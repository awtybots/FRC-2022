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

    private final Color kRedColor, kBlueColor;

    private final ColorSensorV3 sensor;
    private final ColorMatch colorMatch = new ColorMatch();

    public ColorSensor(
            I2C.Port port, Color kRed, Color kBlue, int minDistance, double minConfidence) {
        this.minimumDistance = minDistance;
        this.minColorConfidence = minConfidence;
        this.kRedColor = kRed;
        this.kBlueColor = kBlue;

        colorMatch.addColorMatch(kRed);
        colorMatch.addColorMatch(kBlue);
        colorMatch.setConfidenceThreshold(minColorConfidence);

        sensor = new ColorSensorV3(port);

        sensor.configureColorSensor(
                ColorSensorResolution.kColorSensorRes17bit,
                ColorSensorMeasurementRate.kColorRate25ms,
                GainFactor.kGain3x);

        sensor.configureProximitySensor(
                ProximitySensorResolution.kProxRes11bit,
                ProximitySensorMeasurementRate.kProxRate6ms);
    }

    public boolean ballPresent() {
        return sensor.getProximity() > minimumDistance;
    }

    public Alliance getBallAlliance() {
        Color detectedColor = sensor.getColor();
        ColorMatchResult match = colorMatch.matchColor(detectedColor);

        if (match != null) {
            if (match.color == kRedColor) return Alliance.Red;
            if (match.color == kBlueColor) return Alliance.Blue;
        }

        return Alliance.Invalid;
    }

    private String rgbToString(Color c) {
        return String.format("RGB(%.2f, %.2f, %.2f)", c.red, c.green, c.blue);
    }

    public String rawRGB() {
        return rgbToString(sensor.getColor());
    }

    public int getProximity() {
        return sensor.getProximity();
    }
}
