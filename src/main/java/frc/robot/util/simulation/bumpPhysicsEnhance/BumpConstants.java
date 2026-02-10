package frc.robot.util.simulation.bumpPhysicsEnhance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class BumpConstants {

  // field dimensions
  public static final double FIELD_LENGTH = 16.54; // X
  public static final double FIELD_WIDTH  = 8.07;  // Y

  // bump dimensions
  public static final double BUMP_LENGTH_X = 1.128; // SHORT SIDE（X）
  public static final double BUMP_WIDTH_Y  = 1.854; // LONG SIDE（Y）
  public static final double BUMP_HEIGHT   = 0.1654; // 6.513 in
  public static final double BUMP_SLOPE_RAD =
      Math.toRadians(15.0);

  // bump edge-to-field distances
  public static final double BUMP_EDGE_TO_FIELD_X = 1.584293;
  public static final double BUMP_EDGE_TO_FIELD_Y = 4.065095;

public static final Pose2d[] BUMP_CENTERS = { //TODO PUBLISH VALUES FOR TEST

  // blue 1
  new Pose2d(
      2.148293,
      4.992595,
      Rotation2d.fromDegrees(0)),

  // blue 2
  new Pose2d(
      14.391707,
      4.992595,
      Rotation2d.fromDegrees(180)),

  // red 1
  new Pose2d(
      2.148293,
      3.077405,
      Rotation2d.fromDegrees(0)),

  // red 2
  new Pose2d(
      14.391707,
      3.077405,
      Rotation2d.fromDegrees(180))
};
}