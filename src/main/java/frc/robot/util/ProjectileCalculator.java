package frc.robot.util;

import frc.robot.Constants;

public final class ProjectileCalculator {
  private ProjectileCalculator() {}

  /** Lookup shooter RPS from distance (meters) with zero radial velocity. */
  public static double getStaticShotRps(double distanceMeters) {
    return Interpolating2DMap.lookup(
        Constants.ProjectileConstants.RadialVelocityToDistanceToShooterRps, 0.0, distanceMeters);
  }

  /** Lookup hood angle (deg) from distance (meters) with zero radial velocity. */
  public static double getStaticShotHoodAngle(double distanceMeters) {
    return Interpolating2DMap.lookup(
        Constants.ProjectileConstants.RadialVelocityToDistanceToHoodPositionDegs,
        0.0,
        distanceMeters);
  }

  /** Lookup flight time (seconds) from distance (meters). */
  public static double getStaticShotFlightTime(double distanceMeters) {
    return Constants.ProjectileConstants.DistanceToFlightTimeSecs.get(distanceMeters);
  }

  /** Estimate motion-compensated shot RPS from distance and radial velocity. */
  public static double estimateMotionShotRps(double distanceMeters, double radialVelocity) {
    return Interpolating2DMap.lookup(
        Constants.ProjectileConstants.RadialVelocityToDistanceToShooterRps,
        radialVelocity,
        distanceMeters);
  }

  /** Estimate motion-compensated hood angle (deg) from distance and radial velocity. */
  public static double estimateMotionShotHoodAngle(double distanceMeters, double radialVelocity) {
    return Interpolating2DMap.lookup(
        Constants.ProjectileConstants.RadialVelocityToDistanceToHoodPositionDegs,
        radialVelocity,
        distanceMeters);
  }

  /*
   * Estimate lead yaw angle (degrees) given tangential velocity (m/s), flight time (s), and
   * distance to target (m).
   */
  public static double estimateLeadYawDegrees(double distanceMeters, double tangentialVelocity) {
    if (Math.abs(distanceMeters) <= 1e-6) {
      return 0.0;
    }
    double flightTimeSecs = getStaticShotFlightTime(distanceMeters);
    return Math.toDegrees(Math.atan((tangentialVelocity * flightTimeSecs) / distanceMeters));
  }

  /* BRIEF INTRODUCTION */
  /*
   * Now all the motion shot methods above manages both rps and hood angle.
   * Therefore, the motion shot methods considers both horizontal and vertical velocities.
   * To do this, I created two 2D maps to lookup based on distance & radial velocity.
   * (distance, radialVelocity) -> (rps) & (distance, radialVelocity) -> (hood angle)
   * To optimize this system, you should optimize the static shot trajectories first.
   * For the motion shot part, after optimizing the static shots, you should adjust the constants in the 2D maps.
   * For better motion shot performance, you should change the constants to make the motion shot
   * trajectories at a specifc distance have the same shapes as the static shots have at the given point.
   * The basic logic is that we want to make the velocity vector of the ball at wherever of thefield the same
   * Consequently, tune the constants to make the motion shots and static shots have the same trajectory.
   */
}
