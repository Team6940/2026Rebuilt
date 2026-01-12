package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;

public class MathUtils {

  private MathUtils() {}

  // Translation slew limiters (time-domain smoothing)
  private static final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0); // units per second
  private static final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);

  /**
   * Applies slew rate limiting to a translation vector.
   *
   * <p>This smooths acceleration and deceleration without affecting direction geometry.
   *
   * @param input translation vector after joystick shaping
   * @return slew-limited translation
   */
  public static Translation2d slewTranslation(Translation2d input) {
    return new Translation2d(xLimiter.calculate(input.getX()), yLimiter.calculate(input.getY()));
  }
}
