package frc.robot.util.simulation;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * <h3>Hood Angle to Projectile Launch Angle Interpolation Table</h3>
 * <p>Provides a lookup table for Hood angle input mapping to projectile launch angle output (degrees).
 * The launch angle is consistently offset higher than the hood angle to account for the mechanism's effect.
 * Use for trajectory simulation by interpolating between defined data points.</p>
 *
 * @see TrajectorySimulator
 */
public final class HoodAngleToLaunchAngleInterpolationTable {

  private HoodAngleToLaunchAngleInterpolationTable() {
    throw new AssertionError("Utility class; do not instantiate");
  }

  /** <p>Hood Angle (degrees) → Launch Angle (degrees)</p> */
  public static final InterpolatingDoubleTreeMap TABLE =
      new InterpolatingDoubleTreeMap();

  static {
    TABLE.put(17.490,90.475889);
    TABLE.put(25.049, 74.0);
    TABLE.put(35.31, 71.3099325);
    TABLE.put(35.33, 70.1301024);
    TABLE.put(42.100, 60.8956497);

    // TABLE.put(15.0, 62.0);
    // TABLE.put(18.0, 64.0);
    // TABLE.put(23.0, 67.0);
    // TABLE.put(27.0, 69.0);
    // TABLE.put(33.0, 70.0);
    // TABLE.put(38.0, 70.5);
    // TABLE.put(43.0, 70.5);
    // TABLE.put(50.0, 70.0);
  }
}
