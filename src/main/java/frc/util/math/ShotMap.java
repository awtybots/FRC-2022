package frc.util.math;

import edu.wpi.first.math.MathUtil;
import java.util.Map.Entry;
import java.util.TreeMap;

public class ShotMap {
  private final TreeMap<Double, Double> knownShots = new TreeMap<Double, Double>();

  public ShotMap() {}

  public void addShot(double distance, double shooterSpeed) {
    knownShots.put(distance, shooterSpeed);
  }

  public double calculateShot(double distance) {
    if (knownShots.isEmpty()) {
      System.err.println("shot map is empty!");
      return 0;
    } else if (distance <= knownShots.firstKey()) {
      // shorter distance than in list of shots
      return knownShots.firstEntry().getValue();
    } else if (distance >= knownShots.lastKey()) {
      // longer distance than in list of shots
      return knownShots.lastEntry().getValue();
    }

    // distance is an exact match to a known shot
    Double value = knownShots.get(distance);
    if (value != null) return value;

    Entry<Double, Double> lower = knownShots.floorEntry(distance);
    Entry<Double, Double> upper = knownShots.ceilingEntry(distance);
    double t = (distance - lower.getKey()) / (upper.getKey() - lower.getKey());
    return MathUtil.interpolate(lower.getValue(), upper.getValue(), t);
  }
}
