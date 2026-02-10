package frc.robot.util.simulation.bumpPhysicsEnhance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Pseudo physics util for simulating robot driving over field bumps.
 * Pitch is defined strictly in FIELD (absolute) coordinates.
 */
public final class BumpPhysicsUtil {

  private BumpPhysicsUtil() {}

  /** Output state passed between frames */
  public static final class BumpState {
    public final double z;
    public final Rotation3d rotation;

    public BumpState(double z, Rotation3d rotation) {
      this.z = z;
      this.rotation = rotation;
    }

    public static BumpState flat() {
      return new BumpState(0.0, new Rotation3d());
    }
  }

  /**
   * Main solver.
   *
   * @param pose robot field-relative pose
   * @return bump state (z + pitch rotation only)
   */
  public static BumpState solve(Pose2d pose) {
    for (Pose2d bump : BumpConstants.BUMP_CENTERS) {
      if (isOnBump(pose, bump)) {
        return solveOnBump(pose, bump);
      }
    }
    return BumpState.flat();
  }

  private static boolean isOnBump(Pose2d robot, Pose2d bump) {
    double dx = Math.abs(robot.getX() - bump.getX());
    double dy = Math.abs(robot.getY() - bump.getY());

    return dx <= BumpConstants.BUMP_LENGTH_X / 2.0
        && dy <= BumpConstants.BUMP_WIDTH_Y / 2.0;
  }

  private static BumpState solveOnBump(Pose2d robot, Pose2d bump) {

    // ---------- Z HEIGHT ----------
    double halfLen = BumpConstants.BUMP_LENGTH_X / 2.0;
    double dx = robot.getX() - bump.getX();
    double xNorm = Math.min(Math.abs(dx) / halfLen, 1.0);

    // triangular profile: 0 -> max -> 0
    double z =
        BumpConstants.BUMP_HEIGHT * (1.0 - xNorm);

    // ---------- PITCH (ABSOLUTE FIELD) ----------
    double pitch;
    if (robot.getY() < bump.getY()) {
      // robot is SOUTH of bump center → nose up
      pitch = +BumpConstants.BUMP_SLOPE_RAD;
    } else {
      // robot is NORTH of bump center → nose down
      pitch = -BumpConstants.BUMP_SLOPE_RAD;
    }

    Rotation3d rot = new Rotation3d(0.0, pitch, 0.0);
    return new BumpState(z, rot);
  }
}