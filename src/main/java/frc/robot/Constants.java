// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.IOException;
import java.nio.file.Path;
import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class MotorIDs {

    /*   Chassis   */
    // Pigeon IMU
    public static final int kPigeonId = 0; // TODO CHANGE TO REAL VALUE
    // Front Left
    public static final int kFrontLeftDriveMotorId = 3;
    public static final int kFrontLeftSteerMotorId = 4;
    public static final int kFrontLeftEncoderId = 10;
    // Front Right
    public static final int kFrontRightDriveMotorId = 1;
    public static final int kFrontRightSteerMotorId = 2;
    public static final int kFrontRightEncoderId = 9;
    // Back Left
    public static final int kBackLeftDriveMotorId = 5;
    public static final int kBackLeftSteerMotorId = 6;
    public static final int kBackLeftEncoderId = 11;
    // Back Right
    public static final int kBackRightDriveMotorId = 7;
    public static final int kBackRightSteerMotorId = 8;
    public static final int kBackRightEncoderId = 12;

    /*   Stretcher   */
    public static final int StretcherMotorID = 14;

    /*   Intake   */
    public static final int IntakeMotorID = 15;

    /*   Shooter   */
    public static final int ShooterMotorID = 16;
    public static final int ShooterFollowerMotorID = 19;

    /*   Hood   */
    public static final int HoodMotorID = 17;

    /*   Turret   */
    public static final int TurretMotorID = 18;
  }

  public final class DriveConstants {
    public static final double DEADBAND = 0.1;
    public static final double ANGLE_KP = 5.0;
    public static final double ANGLE_KD = 0.4;
    public static final double ANGLE_MAX_VELOCITY = 8.0;
    public static final double ANGLE_MAX_ACCELERATION = 20.0;
    public static final double FF_START_DELAY = 2.0; // Secs
    public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    public static final double MOVE_TO_X_KP = 5.;
    public static final double MOVE_TO_Y_KP = 5.;
    public static final double MOVE_TO_THETA_KP = 5.;
    public static final double MOVE_TO_X_KD = 0.4;
    public static final double MOVE_TO_Y_KD = 0.4;
    public static final double MOVE_TO_THETA_KD = 0.4;
    public static final double MOVE_TO_POSITION_TOLERANCE_METERS = 0.05;
    public static final double MOVE_TO_ANGLE_TOLERANCE_DEGREES = 3.;

    public static final double PP_TRANSLATION_KP = 5.;
    public static final double PP_TRANSLATION_KD = 0.4;
    public static final double PP_ROTATION_KP = 5.;
    public static final double PP_ROTATION_KD = 0.4;
  }

  public final class ProjectileConstants {
    /** Distance (meters) -> Shooter RPS */
    public static final InterpolatingDoubleTreeMap DistanceToShooterRps =
        new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap DistanceToHoodPositionDegs =
        new InterpolatingDoubleTreeMap();

    /** 2D correction surface: hood angle (deg) -> (radial velocity m/s -> ΔRPS). */
    public static final NavigableMap<Double, InterpolatingDoubleTreeMap> CorrectionSurface =
        new TreeMap<>();

    static {
      // Distance (meters) -> Shooter RPS
      DistanceToShooterRps.put(1.5, 30.0);
      DistanceToShooterRps.put(2.5, 35.0);
      DistanceToShooterRps.put(3.5, 40.0);
      DistanceToShooterRps.put(4.5, 45.0);

      // Distance (meters) -> Hood position (degrees)
      DistanceToHoodPositionDegs.put(1.5, 10.0);
      DistanceToHoodPositionDegs.put(2.5, 18.0);
      DistanceToHoodPositionDegs.put(3.5, 26.0);
      DistanceToHoodPositionDegs.put(4.5, 34.0);

      // Radial Velocity (m/s) -> ΔRPS, the name represents the hood angle
      InterpolatingDoubleTreeMap hood20 = new InterpolatingDoubleTreeMap();
      hood20.put(-1.0, -2.0);
      hood20.put(0.0, 0.0);
      hood20.put(1.0, 2.0);

      InterpolatingDoubleTreeMap hood30 = new InterpolatingDoubleTreeMap();
      hood30.put(-1.0, -3.0);
      hood30.put(0.0, 0.0);
      hood30.put(1.0, 3.0);

      InterpolatingDoubleTreeMap hood40 = new InterpolatingDoubleTreeMap();
      hood40.put(-1.0, -4.0);
      hood40.put(0.0, 0.0);
      hood40.put(1.0, 4.0);

      CorrectionSurface.put(20.0, hood20);
      CorrectionSurface.put(30.0, hood30);
      CorrectionSurface.put(40.0, hood40);
    }
  }

  public static final class ProjectileCalculator {
    /** Lookup shooter RPS from distance, with no motion correction. */
    public static double getStaticShotRps(double distanceMeters) {
      return ProjectileConstants.DistanceToShooterRps.get(distanceMeters);
    }

    /** Lookup hood angle (deg) from distance (meters). */
    public static double getStaticShotHoodAngle(double distanceMeters) {
      return ProjectileConstants.DistanceToHoodPositionDegs.get(distanceMeters);
    }

    /** Bilinear lookup of ΔRPS using hood angle (deg) and radial velocity (m/s). */
    public static double getMotionShotRpsCorrection(double hoodAngleDegs, double radialVelocity) {
      var correctionSurface = ProjectileConstants.CorrectionSurface;
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
  }

  public static final class FieldConstants {
    private static final Path LIBRARY_LAYOUT_PATH =
        Path.of("src", "main", "deploy", "pathplanner", "field2026", "2026-official-andymark.json");
    private static final AprilTagFieldLayout LAYOUT = loadAprilTagLayout();

    // AprilTag related constants
    public static final int aprilTagCount = LAYOUT.getTags().size();
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);
    public static final double fieldLength = LAYOUT.getFieldLength();
    public static final double fieldWidth = LAYOUT.getFieldWidth();

    /**
     * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
     */
    public static class LinesVertical {
      public static final double center = fieldLength / 2.0;
      public static final double starting = LAYOUT.getTagPose(26).get().getX();
      public static final double allianceZone = starting;
      public static final double hubCenter = LAYOUT.getTagPose(26).get().getX() + Hub.width / 2.0;
      public static final double neutralZoneNear = center - Units.inchesToMeters(120);
      public static final double neutralZoneFar = center + Units.inchesToMeters(120);
      public static final double oppHubCenter = LAYOUT.getTagPose(4).get().getX() + Hub.width / 2.0;
      public static final double oppAllianceZone = LAYOUT.getTagPose(10).get().getX();
    }

    /**
     * Officially defined and relevant horizontal lines found on the field (defined by Y-axis
     * offset)
     *
     * <p>NOTE: The field element start and end are always left to right from the perspective of the
     * alliance station
     */
    public static class LinesHorizontal {

      public static final double center = fieldWidth / 2.0;

      // Right of hub
      public static final double rightBumpStart = Hub.nearRightCorner.getY();
      public static final double rightBumpEnd = rightBumpStart - RightBump.width;
      public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
      public static final double rightTrenchOpenEnd = 0;

      // Left of hub
      public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
      public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
      public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
      public static final double leftTrenchOpenStart = fieldWidth;
    }

    /** Hub related constants */
    public static class Hub {

      // Dimensions
      public static final double width = Units.inchesToMeters(47.0);
      public static final double height =
          Units.inchesToMeters(72.0); // includes the catcher at the top
      public static final double innerWidth = Units.inchesToMeters(41.7);
      public static final double innerHeight = Units.inchesToMeters(56.5);

      // Relevant reference points on alliance side
      public static final Translation3d topCenterPoint =
          new Translation3d(
              LAYOUT.getTagPose(26).get().getX() + width / 2.0, fieldWidth / 2.0, height);
      public static final Translation3d innerCenterPoint =
          new Translation3d(
              LAYOUT.getTagPose(26).get().getX() + width / 2.0, fieldWidth / 2.0, innerHeight);
      public static final Translation2d centerPoint =
          new Translation2d(LAYOUT.getTagPose(26).get().getX() + width / 2.0, fieldWidth / 2.0);

      public static final Translation2d nearLeftCorner =
          new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d nearRightCorner =
          new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
      public static final Translation2d farLeftCorner =
          new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d farRightCorner =
          new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

      // Relevant reference points on the opposite side
      public static final Translation3d oppTopCenterPoint =
          new Translation3d(
              LAYOUT.getTagPose(4).get().getX() + width / 2.0, fieldWidth / 2.0, height);
      public static final Translation2d oppCenterPoint =
          new Translation2d(LAYOUT.getTagPose(4).get().getX() + width / 2.0, fieldWidth / 2.0);
      public static final Translation2d oppNearLeftCorner =
          new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d oppNearRightCorner =
          new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
      public static final Translation2d oppFarLeftCorner =
          new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d oppFarRightCorner =
          new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

      // Hub faces
      public static final Pose2d nearFace = LAYOUT.getTagPose(26).get().toPose2d();
      public static final Pose2d farFace = LAYOUT.getTagPose(20).get().toPose2d();
      public static final Pose2d rightFace = LAYOUT.getTagPose(18).get().toPose2d();
      public static final Pose2d leftFace = LAYOUT.getTagPose(21).get().toPose2d();
    }

    /** Left Bump related constants */
    public static class LeftBump {

      // Dimensions
      public static final double width = Units.inchesToMeters(73.0);
      public static final double height = Units.inchesToMeters(6.513);
      public static final double depth = Units.inchesToMeters(44.4);

      // Relevant reference points on alliance side
      public static final Translation2d nearLeftCorner =
          new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
      public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
      public static final Translation2d farLeftCorner =
          new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
      public static final Translation2d farRightCorner = Hub.farLeftCorner;

      // Relevant reference points on opposing side
      public static final Translation2d oppNearLeftCorner =
          new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
      public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
      public static final Translation2d oppFarLeftCorner =
          new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
      public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Right Bump related constants */
    public static class RightBump {
      // Dimensions
      public static final double width = Units.inchesToMeters(73.0);
      public static final double height = Units.inchesToMeters(6.513);
      public static final double depth = Units.inchesToMeters(44.4);

      // Relevant reference points on alliance side
      public static final Translation2d nearLeftCorner =
          new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
      public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
      public static final Translation2d farLeftCorner =
          new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
      public static final Translation2d farRightCorner = Hub.farLeftCorner;

      // Relevant reference points on opposing side
      public static final Translation2d oppNearLeftCorner =
          new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
      public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
      public static final Translation2d oppFarLeftCorner =
          new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
      public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Left Trench related constants */
    public static class LeftTrench {
      // Dimensions
      public static final double width = Units.inchesToMeters(65.65);
      public static final double depth = Units.inchesToMeters(47.0);
      public static final double height = Units.inchesToMeters(40.25);
      public static final double openingWidth = Units.inchesToMeters(50.34);
      public static final double openingHeight = Units.inchesToMeters(22.25);

      // Relevant reference points on alliance side
      public static final Translation3d openingTopLeft =
          new Translation3d(LinesVertical.hubCenter, fieldWidth, openingHeight);
      public static final Translation3d openingTopRight =
          new Translation3d(LinesVertical.hubCenter, fieldWidth - openingWidth, openingHeight);

      // Relevant reference points on opposing side
      public static final Translation3d oppOpeningTopLeft =
          new Translation3d(LinesVertical.oppHubCenter, fieldWidth, openingHeight);
      public static final Translation3d oppOpeningTopRight =
          new Translation3d(LinesVertical.oppHubCenter, fieldWidth - openingWidth, openingHeight);
    }

    public static class RightTrench {

      // Dimensions
      public static final double width = Units.inchesToMeters(65.65);
      public static final double depth = Units.inchesToMeters(47.0);
      public static final double height = Units.inchesToMeters(40.25);
      public static final double openingWidth = Units.inchesToMeters(50.34);
      public static final double openingHeight = Units.inchesToMeters(22.25);

      // Relevant reference points on alliance side
      public static final Translation3d openingTopLeft =
          new Translation3d(LinesVertical.hubCenter, openingWidth, openingHeight);
      public static final Translation3d openingTopRight =
          new Translation3d(LinesVertical.hubCenter, 0, openingHeight);

      // Relevant reference points on opposing side
      public static final Translation3d oppOpeningTopLeft =
          new Translation3d(LinesVertical.oppHubCenter, openingWidth, openingHeight);
      public static final Translation3d oppOpeningTopRight =
          new Translation3d(LinesVertical.oppHubCenter, 0, openingHeight);
    }

    /** Tower related constants */
    public static class Tower {
      // Dimensions
      public static final double width = Units.inchesToMeters(49.25);
      public static final double depth = Units.inchesToMeters(45.0);
      public static final double height = Units.inchesToMeters(78.25);
      public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
      public static final double frontFaceX = Units.inchesToMeters(43.51);

      public static final double uprightHeight = Units.inchesToMeters(72.1);

      // Rung heights from the floor
      public static final double lowRungHeight = Units.inchesToMeters(27.0);
      public static final double midRungHeight = Units.inchesToMeters(45.0);
      public static final double highRungHeight = Units.inchesToMeters(63.0);

      // Relevant reference points on alliance side
      public static final Translation2d centerPoint =
          new Translation2d(frontFaceX, LAYOUT.getTagPose(31).get().getY());
      public static final Translation2d leftUpright =
          new Translation2d(
              frontFaceX,
              (LAYOUT.getTagPose(31).get().getY())
                  + innerOpeningWidth / 2
                  + Units.inchesToMeters(0.75));
      public static final Translation2d rightUpright =
          new Translation2d(
              frontFaceX,
              (LAYOUT.getTagPose(31).get().getY())
                  - innerOpeningWidth / 2
                  - Units.inchesToMeters(0.75));

      // Relevant reference points on opposing side
      public static final Translation2d oppCenterPoint =
          new Translation2d(fieldLength - frontFaceX, LAYOUT.getTagPose(15).get().getY());
      public static final Translation2d oppLeftUpright =
          new Translation2d(
              fieldLength - frontFaceX,
              (LAYOUT.getTagPose(15).get().getY())
                  + innerOpeningWidth / 2
                  + Units.inchesToMeters(0.75));
      public static final Translation2d oppRightUpright =
          new Translation2d(
              fieldLength - frontFaceX,
              (LAYOUT.getTagPose(15).get().getY())
                  - innerOpeningWidth / 2
                  - Units.inchesToMeters(0.75));
    }

    public static class Depot {
      // Dimensions
      public static final double width = Units.inchesToMeters(42.0);
      public static final double depth = Units.inchesToMeters(27.0);
      public static final double height = Units.inchesToMeters(1.125);
      public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

      // Relevant reference points on alliance side
      public static final Translation3d depotCenter =
          new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY, height);
      public static final Translation3d leftCorner =
          new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
      public static final Translation3d rightCorner =
          new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
    }

    public static class Outpost {
      // Dimensions
      public static final double width = Units.inchesToMeters(31.8);
      public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
      public static final double height = Units.inchesToMeters(7.0);

      // Relevant reference points on alliance side
      public static final Translation2d centerPoint =
          new Translation2d(0, LAYOUT.getTagPose(29).get().getY());
    }

    private static AprilTagFieldLayout loadAprilTagLayout() {
      try {
        return new AprilTagFieldLayout(LIBRARY_LAYOUT_PATH);
      } catch (IOException e) {
        throw new RuntimeException("Failed to load AprilTag layout", e);
      }
    }
  }

  public final class IntakeConstants {
    public static final double IntakeRatio = 1.0 / 1.0; // Sensor rotations to mechanism rotations
    public static final InvertedValue IntakeInverted = InvertedValue.Clockwise_Positive;
    public static final double IntakeSupplyCurrentLimit = 40.0;

    // PID Gains
    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.1;
    public static final double kS = 0.0;

    public static final double IntakeVelocityToleranceRPS = 0.5;

    public static final double IntakingRPS = 10.0;
    public static final double ReversingRPS = 0;
  }

  public final class HoodConstants {
    public static final double HoodRatio = 1.0 / 1.0;
    public static final InvertedValue Inverted = InvertedValue.Clockwise_Positive;
    public static final double HoodSupplyCurrentLimit = 40.0;

    // PID Gains
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;
    public static final double kS = 0.0;

    // Motion Magic Gains
    public static final double MaxVelocity = 4.0; // Rotations per second
    public static final double Acceleration = 8.0; // Rotations per second squared

    // Positions (Degrees)
    public static final double HoodPositionToleranceDegs = 2.0;
    public static final double MinDegs = -10.0;
    public static final double MaxDegs = 90.0;
    public static final double IdlePosition = 0.0;

    // Manual control tuning
    public static final double HoodManualSensitivity = 1.0;
    public static final double HoodHybridRangeDegs = 15.0;
  }

  public final class TurretConstants {
    public static final double TurretRatio = 1.0 / 1.0;
    public static final InvertedValue Inverted = InvertedValue.Clockwise_Positive;
    public static final double TurretSupplyCurrentLimit = 40.0;

    // PID Gains
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;
    public static final double kS = 0.0;

    // Motion Magic Gains
    public static final double MaxVelocity = 4.0; // Rotations per second
    public static final double Acceleration = 8.0; // Rotations per second squared

    // Positions (Degrees)
    public static final double TurretPositionToleranceDegs = 2.0;
    public static final double MinDegs = -180.0;
    public static final double MaxDegs = 180.0;
    public static final double IdlePosition = 0.0;

    // Manual control tuning
    public static final double TurretManualSensitivity = 1.0;
    public static final double TurretHybridRangeDegs = 30.0;
  }

  public final class StretcherConstants {
    public static final double StretcherRatio =
        1.0 / 1.0; // Sensor rotations to mechanism rotations
    public static final InvertedValue Inverted = InvertedValue.Clockwise_Positive;
    public static final double StretcherVelocityToleranceRPS = 0.2;
    public static final double StretcherSupplyCurrentLimit = 40.0;

    // PID Gains
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.2;
    public static final double kS = 0.0;
    // public static final double kG = 0.3;

    // Motion Magic Gains
    public static final double MaxVelocity = 4.0; // Rotations per second
    public static final double Acceleration = 8.0; // Rotations per second squared
    // public static final double Deadband = 0.24;

    // Positions(Degrees)
    public static final double StretcherPositionToleranceDegs = 3.;

    public static final double MinDegs = -66.; // degrees CCW Positive
    public static final double MaxDegs = 90.;

    public static final double ExtendedPosition = -61.;
    public static final double RetractedPosition = 90.;

    public static final double IdlePosition = 0.;
  }

  public final class ShooterConstants {
    public static final double ShooterRatio = 1.0 / 1.0;
    public static final InvertedValue Inverted = InvertedValue.Clockwise_Positive;
    public static final double ShooterVelocityToleranceRPS = 0.5;
    public static final MotorAlignmentValue FollowerAlignment = MotorAlignmentValue.Aligned;

    // PID Gains
    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Torque-current feedforward (Amps per RPS)
    public static final double kTorqueFFPerRPS = 0.0;

    // Consider using a static Torque feedforward here
    public static final double kT = 0.1;

    // Current limits
    public static final double ShooterSupplyCurrentLimit = 80.0;
  }
}
