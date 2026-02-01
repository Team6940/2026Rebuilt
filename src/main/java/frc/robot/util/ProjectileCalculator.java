package frc.robot.util;

import frc.robot.Constants;

public final class ProjectileCalculator {
  private ProjectileCalculator() {}

  /** Lookup shooter RPS from distance, with no motion correction. */
  public static double getStaticShotRps(double distanceMeters) {
    return Constants.ProjectileConstants.DistanceToShooterRps.get(distanceMeters);
  }

  /** Lookup hood angle (deg) from distance (meters). */
  public static double getStaticShotHoodAngle(double distanceMeters) {
    return Constants.ProjectileConstants.DistanceToHoodPositionDegs.get(distanceMeters);
  }

  /** Lookup flight time (seconds) from distance (meters). */
  public static double getStaticShotFlightTime(double distanceMeters) {
    return Constants.ProjectileConstants.DistanceToFlightTimeSecs.get(distanceMeters);
  }

  /** Estimate horizontal ball velocity (m/s) from distance and flight time. */
  public static double estimateStaticShotVx(double distanceMeters) {
    double flightTimeSecs = getStaticShotFlightTime(distanceMeters);
    if (flightTimeSecs <= 1e-6) {
      return 0.0;
    }
    return distanceMeters / flightTimeSecs;
  }

  /**
   * Estimate motion-compensated shot RPS from static shot RPS and velocities.
   *
   * <p>Positive radial velocity means approaching the target (less exit velocity needed).
   */
  public static double estimateMotionShotRps(
      double staticShotRps, double staticShotVx, double radialVelocity) {
    if (staticShotVx <= 1e-6) {
      return staticShotRps;
    }

    double adjustedVx = staticShotVx - radialVelocity;
    if (adjustedVx < 0.0) {
      adjustedVx = 0.0;
    }

    return staticShotRps * (adjustedVx / staticShotVx);
  }

  /** Estimate motion-compensated shot RPS from distance and radial velocity. */
  public static double estimateMotionShotRps(double distanceMeters, double radialVelocity) {
    double staticShotRps = getStaticShotRps(distanceMeters);
    double staticShotVx = estimateStaticShotVx(distanceMeters);
    return estimateMotionShotRps(staticShotRps, staticShotVx, radialVelocity);
  }

  /*
   * Estimate lead yaw angle (degrees) given tangential velocity (m/s), flight time (s), and
   * distance to target (m).
   */
  public static double estimateLeadYawDegrees(double tangentialVelocity, double distanceMeters) {
    if (Math.abs(distanceMeters) <= 1e-6) {
      return 0.0;
    }
    double flightTimeSecs = getStaticShotFlightTime(distanceMeters);
    return Math.toDegrees(Math.atan((tangentialVelocity * flightTimeSecs) / distanceMeters));
  }

  /**
   * Bilinear lookup of ΔRPS using hood angle (deg) and radial velocity (m/s). This correction
   * method uses linear interpolation between nearest neighbors in a 2D grid. Tuning the 2D table is
   * done empirically to account for robot motion effects on shot velocity. Do NOT use this method
   * if calculating flight time is considered a better approach.
   */
  public static double estimateMotionShotRpsCorrection2DTable(
      double hoodAngleDegs, double radialVelocity) {
    var correctionSurface = Constants.ProjectileConstants.CorrectionSurface;
    if (correctionSurface.isEmpty()) {
      return 0.0;
    }

    var lowerEntry = correctionSurface.floorEntry(hoodAngleDegs);
    var upperEntry = correctionSurface.ceilingEntry(hoodAngleDegs);

    if (lowerEntry == null) {
      lowerEntry = correctionSurface.firstEntry();
    }
    if (upperEntry == null) {
      upperEntry = correctionSurface.lastEntry();
    }

    double lowAngle = lowerEntry.getKey();
    double highAngle = upperEntry.getKey();

    double deltaLow = lowerEntry.getValue().get(radialVelocity);
    double deltaHigh = upperEntry.getValue().get(radialVelocity);

    if (Math.abs(highAngle - lowAngle) < 1e-6) {
      return deltaLow;
    }

    // Linear interpolation
    double t = (hoodAngleDegs - lowAngle) / (highAngle - lowAngle);
    return deltaLow + t * (deltaHigh - deltaLow);
  }

  /*
   * All the motion shot methods above do NOT consider vertical velocity effects.
   * So choosing the best static shot trajectories are extremely important to minimize vertical error.
   * If vertical velocity is significant, consider using physical models to calculate required exit velocity and angle.
   * e.g., use interpolation tables which converts (HoodDegs, ShootRps) -> (ExitVelocity, ExitAngle)
   * Well, this needs to be determined later.
   */
}
