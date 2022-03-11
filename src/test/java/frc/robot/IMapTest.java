package frc.robot;

import frc.robot.util.math.Convert;
import frc.robot.util.math.Interpolatable;
import frc.robot.util.math.InterpolationMap;

public class IMapTest {
  private static final InterpolationMap<Double> iMap = new InterpolationMap<>();
  
  private static void initIMap() {
    iMap.addKeyframe(Convert.feetToMeters(5.0, 6.0), Interpolatable.interpolatableDouble(2000));
    iMap.addKeyframe(Convert.feetToMeters(6.0, 6.0), Interpolatable.interpolatableDouble(2500));
    iMap.addKeyframe(Convert.feetToMeters(7.0, 6.0), Interpolatable.interpolatableDouble(3000));
  }

  public static void main(String[] args) {
    initIMap();
    System.out.println(iMap.get(Convert.feetToMeters(3.0, 0.0)));
    System.out.println(iMap.get(Convert.feetToMeters(6.0, 0.0)));
    System.out.println(iMap.get(Convert.feetToMeters(10.0, 0.0)));
  }
}
