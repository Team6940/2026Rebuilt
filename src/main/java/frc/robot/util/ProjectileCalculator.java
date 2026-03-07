package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public final class ProjectileCalculator {
  private ProjectileCalculator() {}

  // ─────────────────────────────────────────────────────────────────────────
  // Shared static-shot lookups
  // ─────────────────────────────────────────────────────────────────────────

  /** Lookup shooter RPS from distance (meters) at zero radial velocity (static shot). */
  public static double getStaticShotRps(double distanceMeters) {
    return Interpolating2DMap.lookup(
        Constants.ProjectileConstants.RadialVelocityToDistanceToShooterRps, 0.0, distanceMeters);
  }

  /** Lookup hood angle (deg) from distance (meters) at zero radial velocity (static shot). */
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

  // ─────────────────────────────────────────────────────────────────────────
  // Method A – Direct 2D map lookup (original)
  //
  // Pre-compute radialVelocity / tangentialVelocity yourself (e.g. from
  // Drive.getHubRelativeChassisSpeeds()) and pass them in.  Fast and simple,
  // but uses the current distance — NOT where the turret will be on landing.
  // ─────────────────────────────────────────────────────────────────────────

  /**
   * [Method A] Estimate motion-compensated shooter RPS via direct 2D map lookup.
   *
   * @param distanceMeters current turret-to-target distance (m)
   * @param radialVelocity radial component of turret velocity (m/s, positive = moving away)
   */
  public static double estimateMotionShotRps_Direct(double distanceMeters, double radialVelocity) {
    return Interpolating2DMap.lookup(
        Constants.ProjectileConstants.RadialVelocityToDistanceToShooterRps,
        radialVelocity,
        distanceMeters);
  }

  /**
   * [Method A] Estimate motion-compensated hood angle (deg) via direct 2D map lookup.
   *
   * @param distanceMeters current turret-to-target distance (m)
   * @param radialVelocity radial component of turret velocity (m/s, positive = moving away)
   */
  public static double estimateMotionShotHoodAngle_Direct(
      double distanceMeters, double radialVelocity) {
    return Interpolating2DMap.lookup(
        Constants.ProjectileConstants.RadialVelocityToDistanceToHoodPositionDegs,
        radialVelocity,
        distanceMeters);
  }

  /**
   * [Method A] Estimate turret lead yaw angle (degrees) via direct trig.
   *
   * @param distanceMeters current turret-to-target distance (m)
   * @param tangentialVelocity tangential component of turret velocity (m/s, CCW positive)
   */
  public static double estimateLeadYawDegrees_Direct(
      double distanceMeters, double tangentialVelocity) {
    if (Math.abs(distanceMeters) <= 1e-6) return 0.0;
    double tof = getStaticShotFlightTime(distanceMeters);
    return Math.toDegrees(Math.atan((tangentialVelocity * tof) / distanceMeters));
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Method B – Iterative lookahead solver
  //
  // Finds the "virtual target": the point the turret must aim at so the note
  // lands on the real target despite robot motion.  RPS and hood are then
  // looked up from the static maps at the lookahead distance — no separate
  // radial-velocity adjustment needed, because the geometry already encodes
  // the compensation.
  //
  // Usage in HybridShootCommand:
  //   ShotSolution sol = ProjectileCalculator.solve(
  //       drive.getTurretWorldPosition(), hubCenter, turretVx, turretVy);
  //   turret.setAutoSetpointFieldRelativeRotation2d(sol.turretAimAngle(), pose);
  //   hood.setAutoSetpoint(sol.hoodAngleDeg());
  //   shooter.setRPS(sol.shooterRps());
  // ─────────────────────────────────────────────────────────────────────────

  private static final int LOOKAHEAD_ITERATIONS = 20;

  /**
   * All output values needed for a single shot, produced by {@link #solve}.
   *
   * @param virtualTarget the field-relative point the turret should aim at (real target displaced
   *     by robot motion during flight)
   * @param turretAimAngle the field-relative angle the turret must point toward the virtual target
   * @param lookaheadDistance distance from the current turret position to the virtual target (m) —
   *     used internally for RPS/hood lookup, available for logging
   * @param shooterRps shooter speed looked up at the lookahead distance (static map)
   * @param hoodAngleDeg hood angle (deg) looked up at the lookahead distance (static map)
   * @param flightTimeSecs converged note flight time (s)
   */
  public record ShotSolution(
      Translation2d virtualTarget,
      Rotation2d turretAimAngle,
      double lookaheadDistance,
      double shooterRps,
      double hoodAngleDeg,
      double flightTimeSecs) {}

  /**
   * [Method B] Iteratively solve for the virtual target and shot parameters.
   *
   * <p>Algorithm (repeated {@value #LOOKAHEAD_ITERATIONS} times until convergence):
   *
   * <ol>
   *   <li>Look up flight time for the current effective distance.
   *   <li>Project the turret position forward by {@code velocity × flightTime} to the position it
   *       will occupy when the note arrives.
   *   <li>Recompute the distance from that projected position to the real target.
   * </ol>
   *
   * <p>The final projected position becomes the <em>virtual target</em>: the turret aims at this
   * point, but the note actually flies to the real target because the robot has moved by then. RPS
   * and hood angle are then read from the static-shot maps at the converged distance — no
   * radial-velocity 2D map is needed.
   *
   * @param turretPosition field-relative turret pivot ({@code Drive.getTurretWorldPosition()})
   * @param realTarget field-relative real target (hub center, tower, etc.)
   * @param turretFieldVelocity field-frame turret pivot velocity (m/s) — must include omega×r
   *     contribution ({@code Drive.getTurretFieldVelocity()})
   * @return {@link ShotSolution} with virtual target, aim angle, RPS, hood angle, and flight time
   */
  public static ShotSolution solve(
      Translation2d turretPosition, Translation2d realTarget, Translation2d turretFieldVelocity) {
    return solve(
        turretPosition, realTarget, turretFieldVelocity.getX(), turretFieldVelocity.getY());
  }

  /**
   * [Method B] Iteratively solve for the virtual target and shot parameters.
   *
   * <p>Scalar-component overload of {@link #solve(Translation2d, Translation2d, Translation2d)}.
   *
   * @param turretPosition field-relative turret pivot ({@code Drive.getTurretWorldPosition()})
   * @param realTarget field-relative real target (hub center, tower, etc.)
   * @param fieldVx turret field-frame X velocity (m/s) — must include omega×r contribution
   * @param fieldVy turret field-frame Y velocity (m/s)
   * @return {@link ShotSolution} with virtual target, aim angle, RPS, hood angle, and flight time
   */
  public static ShotSolution solve(
      Translation2d turretPosition, Translation2d realTarget, double fieldVx, double fieldVy) {

    double lookaheadDistance = turretPosition.getDistance(realTarget);

    for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
      double tof = getStaticShotFlightTime(lookaheadDistance);
      double lx = turretPosition.getX() + fieldVx * tof;
      double ly = turretPosition.getY() + fieldVy * tof;
      lookaheadDistance = Math.hypot(lx - realTarget.getX(), ly - realTarget.getY());
    }

    double tof = getStaticShotFlightTime(lookaheadDistance);

    // Virtual target: where the real target appears to be from the turret's future position.
    // The turret aims here now; the note meets the real target after the robot has moved.
    double virtualX = realTarget.getX() - fieldVx * tof;
    double virtualY = realTarget.getY() - fieldVy * tof;
    Translation2d virtualTarget = new Translation2d(virtualX, virtualY);

    Rotation2d aimAngle = virtualTarget.minus(turretPosition).getAngle();
    double rps = getStaticShotRps(lookaheadDistance);
    double hoodDeg = getStaticShotHoodAngle(lookaheadDistance);

    return new ShotSolution(virtualTarget, aimAngle, lookaheadDistance, rps, hoodDeg, tof);
  }
}
