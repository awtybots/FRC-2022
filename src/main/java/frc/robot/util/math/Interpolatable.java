package frc.robot.util.math;

import edu.wpi.first.math.Pair;

public abstract class Interpolatable<T> {
  private final T value;

  public Interpolatable(T value) {
    this.value = value;
  }

  public T get() {
    return this.value;
  }

  public abstract T interpolate(T endValue, double t);

  public T interpolate(Interpolatable<T> endValue, double t) {
    return interpolate(endValue.value, t);
  }

  public static Interpolatable<Double> interpolatableDouble(double value) {
    return new Interpolatable<Double>(value) {
      @Override
      public Double interpolate(Double endValue, double t) {
        return super.value + t * (endValue - super.value);
      }
    };
  }

  public static Interpolatable<Pair<Double, Double>> interpolatableDoublePair(double a, double b) {
    return new Interpolatable<Pair<Double, Double>>(new Pair<Double, Double>(a, b)) {
      @Override
      public Pair<Double, Double> interpolate(Pair<Double, Double> endValue, double t) {
        return new Pair<Double, Double>(
            super.value.getFirst() + t * (endValue.getFirst() - super.value.getFirst()),
            super.value.getSecond() + t * (endValue.getSecond() - super.value.getSecond()));
      }
    };
  }
}
