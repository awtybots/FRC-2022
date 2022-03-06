package frc.robot.util.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private double mountingHeight;
  private double mountingAngle;

  private final NetworkTable netTable = NetworkTableInstance.getDefault().getTable("limelight");

  public Limelight(double mountingHeight, double mountingAngle) {
    this.mountingAngle = mountingAngle;
    this.mountingHeight = mountingHeight;
  }

  public double getMountingHeight() {
    return mountingHeight;
  }

  public double getMountingAngle() {
    return mountingAngle;
  }

  public boolean hasVisibleTarget() {
    double res = getValue(TableEntry.HasValidTargets);
    return res == 1.0;
  }

  /**
   * @return the horizontal angle (between -29.8deg and 29.8deg) from crosshair to center of target
   */
  public double targetXOffset() {
    return getValue(TableEntry.TargetXOffset);
  }

  /**
   * @return the vertical angle (between -24.85deg and 24.85deg) from crosshair to center of target
   */
  public double targetYOffset() {
    return getValue(TableEntry.TargetYOffset);
  }

  /** Set the current camera pipeline (integer from 0 to 9, inclusive) */
  public void setPipeline(int pipeline) {
    if (pipeline < 10 && pipeline > -1) setValue(TableEntry.CurrentPipeline, pipeline);
  }

  public void toggleDriverMode(boolean enabled) {
    setValue(TableEntry.OperationMode, enabled ? 1.0 : 0.0);
  }

  public void toggleLED(LEDMode state) {
    setValue(TableEntry.LEDMode, state.ordinal());
  }

  private double getValue(TableEntry entry) {
    NetworkTableValue v = netTable.getEntry(entry.getter).getValue();
    if (v.getType() == NetworkTableType.kDouble) {
      return v.getDouble();
    } else {
      return -1.0;
    }
  }

  private boolean setValue(TableEntry entry, double value) {
    if (entry.setter == "") return false;
    return netTable.getEntry(entry.setter).setDouble(value);
  }

  public enum LEDMode {
    /** Uses the default mode set in the active Pipeline */
    PipelineDefault,
    Off,
    Blinking,
    On
  }

  private enum TableEntry {
    HasValidTargets("tv"),
    TargetXOffset("tx"),
    TargetYOffset("ty"),
    TargetArea("ta"),
    TargetSkew("ts"),
    PipelineLatency("tl"),
    CurrentPipeline("getpipe", "pipeline"),
    OperationMode("camMode", "camMode"),
    LEDMode("LEDMode", "LEDMode");
    public String setter, getter;

    TableEntry(String getter, String setter) {
      this.setter = setter;
      this.getter = getter;
    }

    TableEntry(String getter) {
      this.setter = "";
      this.getter = getter;
    }
  }
}
