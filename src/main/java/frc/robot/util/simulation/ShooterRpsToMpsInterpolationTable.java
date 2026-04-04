package frc.robot.util.simulation;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * <h3>Shooter RPS to Projectile MPS Interpolation Tables</h3>
 * <p>Provides three calibrated lookup tables for RPS input mapping to projectile velocity output (Meters per sec).
 * <p>Use for trajectory simulation by interpolating between defined data points.</p>
 *
 * @see TrajectorySimulator
 */
public final class ShooterRpsToMpsInterpolationTable {

  private ShooterRpsToMpsInterpolationTable() {
    throw new AssertionError("Utility class; do not instsantiate");
  }

  /** <p>Preset calibration 1: DEFAULT.</p> */
  public static final InterpolatingDoubleTreeMap PRESET_1 =
      new InterpolatingDoubleTreeMap();

  static {
    PRESET_1.put(20.0, 2.79508);
    // PRESET_1.put(30.0, 11.18);
    // PRESET_1.put(40.0, 13.46);
    PRESET_1.put(55.0, 8.27647);
    // PRESET_1.put(60.0, 18.2);
    PRESET_1.put(70.0, 11.85854);

    // PRESET_1.put(20.0, 3.484);
    // PRESET_1.put(30.0, 5.226);
    // PRESET_1.put(34.5, 6.019);
    // PRESET_1.put(39.84, 6.955);
    // PRESET_1.put(43.8, 7.644);
    // PRESET_1.put(48.7, 8.489);
    // PRESET_1.put(53.5, 9.334);
    // PRESET_1.put(54.5, 9.503);
    // PRESET_1.put(60.0, 10.465);
    // PRESET_1.put(100.0, 17.459);
    // PRESET_1.put(120.0, 20.943);
    
  }

  /** <p>Preset calibration 2</p> */
  public static final InterpolatingDoubleTreeMap PRESET_2 =
      new InterpolatingDoubleTreeMap();

  static {
    PRESET_2.put(20.0, 3.328);
    PRESET_2.put(30.0, 4.992);
    PRESET_2.put(34.5, 5.733);
    PRESET_2.put(39.84, 6.63);
    PRESET_2.put(43.8, 7.293);
    PRESET_2.put(48.7, 8.099);
    PRESET_2.put(53.5, 8.905);
    PRESET_2.put(54.5, 9.061);
    PRESET_2.put(60.0, 9.984);
    PRESET_2.put(100.0, 16.653);
    PRESET_2.put(120.0, 19.994);
  }

  /** <p>Preset calibration 3.</p> */
  public static final InterpolatingDoubleTreeMap PRESET_3 =
      new InterpolatingDoubleTreeMap();

  static {
    PRESET_3.put(20.0, 3.159);
    PRESET_3.put(30.0, 4.745);
    PRESET_3.put(34.5, 5.46);
    PRESET_3.put(39.84, 6.318);
    PRESET_3.put(43.8, 6.942);
    PRESET_3.put(48.7, 7.709);
    PRESET_3.put(53.5, 8.476);
    PRESET_3.put(54.5, 8.632);
    PRESET_3.put(60.0, 9.503);
    PRESET_3.put(100.0, 15.873);
    PRESET_3.put(120.0, 19.058);
  }

  /**
   * <p>Get the interpolation table for the specified preset.</p>
   *
   * @param preset The preset index (1, 2, or 3)
   * @return The corresponding interpolation table, or PRESET_2 if preset is invalid
   */
  public static InterpolatingDoubleTreeMap getPreset(int preset) {
    return switch (preset) {
      case 2 -> PRESET_2;
      case 3 -> PRESET_3;
      default -> PRESET_1;
    };
  }
}
