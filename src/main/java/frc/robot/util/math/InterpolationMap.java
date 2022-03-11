package frc.robot.util.math;

import java.util.Map.Entry;
import java.util.TreeMap;

public class InterpolationMap<T> {
  private TreeMap<Double, Interpolatable<T>> keyframes = new TreeMap<>();

  public InterpolationMap() {}

  public void addKeyframe(double position, Interpolatable<T> value) {
    keyframes.put(position, value);
  }

  public T get(double t) {
    if (keyframes.isEmpty()) {
      return null;
    } else if (t <= keyframes.firstKey()) {
      return keyframes.firstEntry().getValue().get();
    } else if (t >= keyframes.lastKey()) {
      return keyframes.lastEntry().getValue().get();
    }

    Interpolatable<T> value = keyframes.get(t);
    if (value != null) return value.get();

    Entry<Double, Interpolatable<T>> a = keyframes.floorEntry(t);
    Entry<Double, Interpolatable<T>> b = keyframes.ceilingEntry(t);

    return a.getValue().interpolate(b.getValue(), (t - a.getKey()) / (b.getKey() - a.getKey()));
  }
}
