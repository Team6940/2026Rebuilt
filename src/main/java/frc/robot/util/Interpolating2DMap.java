package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Map;
import java.util.NavigableMap;

/**
 * Utility for 2D interpolation/extrapolation over a surface defined as radialKey -> (distance ->
 * value).
 */
public final class Interpolating2DMap {
  private Interpolating2DMap() {}

  /**
   * Lookup a value using a 2D map (outer key -> inner interpolating map). Performs linear
   * interpolation between nearest outer keys. If the query is outside the outer key range, linearly
   * extrapolates using the nearest two keys.
   */
  public static double lookup(
      NavigableMap<Double, InterpolatingDoubleTreeMap> surface, double outerKey, double innerKey) {
    if (surface.isEmpty()) {
      return 0.0;
    }

    Map.Entry<Double, InterpolatingDoubleTreeMap> lower = surface.floorEntry(outerKey);
    Map.Entry<Double, InterpolatingDoubleTreeMap> upper = surface.ceilingEntry(outerKey);

    // Interpolation: outerKey is within range, use nearest lower/upper entries.
    if (lower == null || upper == null) {
      // Extrapolation: outerKey is outside range. Use the nearest two keys.
      if (outerKey < surface.firstKey()) {
        lower = surface.firstEntry();
        upper = surface.higherEntry(lower.getKey());
      } else {
        upper = surface.lastEntry();
        lower = surface.lowerEntry(upper.getKey());
      }
    }

    if (lower.getKey().equals(upper.getKey())) {
      return lower.getValue().get(innerKey);
    }

    double lowKey = lower.getKey();
    double highKey = upper.getKey();
    double lowVal = lower.getValue().get(innerKey);
    double highVal = upper.getValue().get(innerKey);

    // Linear interpolation/extrapolation in the outer dimension.
    double t = (outerKey - lowKey) / (highKey - lowKey);
    return lowVal + t * (highVal - lowVal);
  }
}
