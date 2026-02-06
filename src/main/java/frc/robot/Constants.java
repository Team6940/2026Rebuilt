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
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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
    public static final int kPigeonId = 0;
    // Front Left
    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontLeftSteerMotorId = 2;
    public static final int kFrontLeftEncoderId = 9;
    // Front Right
    public static final int kFrontRightDriveMotorId = 3;
    public static final int kFrontRightSteerMotorId = 4;
    public static final int kFrontRightEncoderId = 10;
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
    public static final int TurretEncoderID = 20;
    public static final int TurretEncoder2ID = 22;

    /*   Feeder   */
    public static final int FeederTurntableMotorID = 20;
    public static final int FeederFeedMotorID = 21;
  }

  public final class DriveConstants {
    public static final double DEADBAND = 0.05;
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
    /** Radial velocity (m/s) -> (distance (m) -> shooter RPS). */
    public static final NavigableMap<Double, InterpolatingDoubleTreeMap>
        RadialVelocityToDistanceToShooterRps = new TreeMap<>();

    /** Radial velocity (m/s) -> (distance (m) -> hood position degs). */
    public static final NavigableMap<Double, InterpolatingDoubleTreeMap>
        RadialVelocityToDistanceToHoodPositionDegs = new TreeMap<>();

    /** Distance (meters) -> Flight time (seconds). */
    public static final InterpolatingDoubleTreeMap DistanceToFlightTimeSecs =
        new InterpolatingDoubleTreeMap();

    static {
      // Radial velocity (m/s) -> Distance (meters) -> Shooter RPS
      InterpolatingDoubleTreeMap shooterRpsNeg1 = new InterpolatingDoubleTreeMap();
      shooterRpsNeg1.put(1.5, 32.0);
      shooterRpsNeg1.put(2.5, 37.0);
      shooterRpsNeg1.put(3.5, 42.0);
      shooterRpsNeg1.put(4.5, 47.0);

      InterpolatingDoubleTreeMap shooterRpsNeg2 = new InterpolatingDoubleTreeMap();
      shooterRpsNeg2.put(1.5, 34.0);
      shooterRpsNeg2.put(2.5, 39.0);
      shooterRpsNeg2.put(3.5, 44.0);
      shooterRpsNeg2.put(4.5, 49.0);

      InterpolatingDoubleTreeMap shooterRpsNeg3 = new InterpolatingDoubleTreeMap();
      shooterRpsNeg3.put(1.5, 36.0);
      shooterRpsNeg3.put(2.5, 41.0);
      shooterRpsNeg3.put(3.5, 46.0);
      shooterRpsNeg3.put(4.5, 51.0);

      InterpolatingDoubleTreeMap shooterRpsZero = new InterpolatingDoubleTreeMap();
      shooterRpsZero.put(1.5, 30.0);
      shooterRpsZero.put(2.5, 35.0);
      shooterRpsZero.put(3.5, 40.0);
      shooterRpsZero.put(4.5, 45.0);

      InterpolatingDoubleTreeMap shooterRpsPos1 = new InterpolatingDoubleTreeMap();
      shooterRpsPos1.put(1.5, 28.0);
      shooterRpsPos1.put(2.5, 33.0);
      shooterRpsPos1.put(3.5, 38.0);
      shooterRpsPos1.put(4.5, 43.0);

      InterpolatingDoubleTreeMap shooterRpsPos2 = new InterpolatingDoubleTreeMap();
      shooterRpsPos2.put(1.5, 26.0);
      shooterRpsPos2.put(2.5, 31.0);
      shooterRpsPos2.put(3.5, 36.0);
      shooterRpsPos2.put(4.5, 41.0);

      InterpolatingDoubleTreeMap shooterRpsPos3 = new InterpolatingDoubleTreeMap();
      shooterRpsPos3.put(1.5, 24.0);
      shooterRpsPos3.put(2.5, 29.0);
      shooterRpsPos3.put(3.5, 34.0);
      shooterRpsPos3.put(4.5, 39.0);

      RadialVelocityToDistanceToShooterRps.put(-3.0, shooterRpsNeg3);
      RadialVelocityToDistanceToShooterRps.put(-2.0, shooterRpsNeg2);
      RadialVelocityToDistanceToShooterRps.put(-1.0, shooterRpsNeg1);
      RadialVelocityToDistanceToShooterRps.put(0.0, shooterRpsZero);
      RadialVelocityToDistanceToShooterRps.put(1.0, shooterRpsPos1);
      RadialVelocityToDistanceToShooterRps.put(2.0, shooterRpsPos2);
      RadialVelocityToDistanceToShooterRps.put(3.0, shooterRpsPos3);

      // Radial velocity (m/s) -> Distance (meters) -> Hood position (degrees)
      InterpolatingDoubleTreeMap hoodNeg1 = new InterpolatingDoubleTreeMap();
      hoodNeg1.put(1.5, 12.0);
      hoodNeg1.put(2.5, 20.0);
      hoodNeg1.put(3.5, 28.0);
      hoodNeg1.put(4.5, 36.0);

      InterpolatingDoubleTreeMap hoodNeg2 = new InterpolatingDoubleTreeMap();
      hoodNeg2.put(1.5, 14.0);
      hoodNeg2.put(2.5, 22.0);
      hoodNeg2.put(3.5, 30.0);
      hoodNeg2.put(4.5, 38.0);

      InterpolatingDoubleTreeMap hoodNeg3 = new InterpolatingDoubleTreeMap();
      hoodNeg3.put(1.5, 16.0);
      hoodNeg3.put(2.5, 24.0);
      hoodNeg3.put(3.5, 32.0);
      hoodNeg3.put(4.5, 40.0);

      InterpolatingDoubleTreeMap hoodZero = new InterpolatingDoubleTreeMap();
      hoodZero.put(1.5, 10.0);
      hoodZero.put(2.5, 18.0);
      hoodZero.put(3.5, 26.0);
      hoodZero.put(4.5, 34.0);

      InterpolatingDoubleTreeMap hoodPos1 = new InterpolatingDoubleTreeMap();
      hoodPos1.put(1.5, 9.0);
      hoodPos1.put(2.5, 17.0);
      hoodPos1.put(3.5, 25.0);
      hoodPos1.put(4.5, 33.0);

      InterpolatingDoubleTreeMap hoodPos2 = new InterpolatingDoubleTreeMap();
      hoodPos2.put(1.5, 8.0);
      hoodPos2.put(2.5, 16.0);
      hoodPos2.put(3.5, 24.0);
      hoodPos2.put(4.5, 32.0);

      InterpolatingDoubleTreeMap hoodPos3 = new InterpolatingDoubleTreeMap();
      hoodPos3.put(1.5, 7.0);
      hoodPos3.put(2.5, 15.0);
      hoodPos3.put(3.5, 23.0);
      hoodPos3.put(4.5, 31.0);
      RadialVelocityToDistanceToHoodPositionDegs.put(-3.0, hoodNeg3);
      RadialVelocityToDistanceToHoodPositionDegs.put(-2.0, hoodNeg2);
      RadialVelocityToDistanceToHoodPositionDegs.put(-1.0, hoodNeg1);
      RadialVelocityToDistanceToHoodPositionDegs.put(0.0, hoodZero);
      RadialVelocityToDistanceToHoodPositionDegs.put(1.0, hoodPos1);
      RadialVelocityToDistanceToHoodPositionDegs.put(2.0, hoodPos2);
      RadialVelocityToDistanceToHoodPositionDegs.put(3.0, hoodPos3);

      // Distance (meters) -> Flight time (seconds)
      DistanceToFlightTimeSecs.put(1.5, 0.45);
      DistanceToFlightTimeSecs.put(2.5, 0.55);
      DistanceToFlightTimeSecs.put(3.5, 0.65);
      DistanceToFlightTimeSecs.put(4.5, 0.75);
    }
  }

  public static final class FieldConstants {
    private static final Path LIBRARY_LAYOUT_PATH =
        Filesystem.getDeployDirectory()
            .toPath()
            .resolve(Path.of("pathplanner", "field2026", "2026-official-andymark.json"));
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
      public static final double starting = getTagPoseOrDefault(26).getX();
      public static final double allianceZone = starting;
      public static final double hubCenter = getTagPoseOrDefault(26).getX() + Hub.width / 2.0;
      public static final double neutralZoneNear = center - Units.inchesToMeters(120);
      public static final double neutralZoneFar = center + Units.inchesToMeters(120);
      public static final double oppHubCenter = getTagPoseOrDefault(4).getX() + Hub.width / 2.0;
      public static final double oppAllianceZone = getTagPoseOrDefault(10).getX();
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
          new Translation3d(getTagPoseOrDefault(26).getX() + width / 2.0, fieldWidth / 2.0, height);
      public static final Translation3d innerCenterPoint =
          new Translation3d(
              getTagPoseOrDefault(26).getX() + width / 2.0, fieldWidth / 2.0, innerHeight);
      public static final Translation2d centerPoint =
          new Translation2d(getTagPoseOrDefault(26).getX() + width / 2.0, fieldWidth / 2.0);

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
          new Translation3d(getTagPoseOrDefault(4).getX() + width / 2.0, fieldWidth / 2.0, height);
      public static final Translation2d oppCenterPoint =
          new Translation2d(getTagPoseOrDefault(4).getX() + width / 2.0, fieldWidth / 2.0);
      public static final Translation2d oppNearLeftCorner =
          new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d oppNearRightCorner =
          new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
      public static final Translation2d oppFarLeftCorner =
          new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d oppFarRightCorner =
          new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

      // Hub faces
      public static final Pose2d nearFace = getTagPoseOrDefault(26).toPose2d();
      public static final Pose2d farFace = getTagPoseOrDefault(20).toPose2d();
      public static final Pose2d rightFace = getTagPoseOrDefault(18).toPose2d();
      public static final Pose2d leftFace = getTagPoseOrDefault(21).toPose2d();
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
          new Translation2d(frontFaceX, getTagPoseOrDefault(31).getY());
      public static final Translation2d leftUpright =
          new Translation2d(
              frontFaceX,
              (getTagPoseOrDefault(31).getY())
                  + innerOpeningWidth / 2
                  + Units.inchesToMeters(0.75));
      public static final Translation2d rightUpright =
          new Translation2d(
              frontFaceX,
              (getTagPoseOrDefault(31).getY())
                  - innerOpeningWidth / 2
                  - Units.inchesToMeters(0.75));

      // Relevant reference points on opposing side
      public static final Translation2d oppCenterPoint =
          new Translation2d(fieldLength - frontFaceX, getTagPoseOrDefault(15).getY());
      public static final Translation2d oppLeftUpright =
          new Translation2d(
              fieldLength - frontFaceX,
              (getTagPoseOrDefault(15).getY())
                  + innerOpeningWidth / 2
                  + Units.inchesToMeters(0.75));
      public static final Translation2d oppRightUpright =
          new Translation2d(
              fieldLength - frontFaceX,
              (getTagPoseOrDefault(15).getY())
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
          new Translation2d(0, getTagPoseOrDefault(29).getY());
    }

    private static Pose3d getTagPoseOrDefault(int id) {
      return LAYOUT
          .getTagPose(id)
          .orElseGet(
              () -> {
                DriverStation.reportError(
                    "Missing AprilTag ID " + id + " in layout: " + LIBRARY_LAYOUT_PATH, false);
                return new Pose3d();
              });
    }

    private static AprilTagFieldLayout loadAprilTagLayout() {
      try {
        return new AprilTagFieldLayout(LIBRARY_LAYOUT_PATH);
      } catch (IOException e) {
        DriverStation.reportError(
            "Failed to load AprilTag layout from " + LIBRARY_LAYOUT_PATH, e.getStackTrace());
        return new AprilTagFieldLayout(new java.util.ArrayList<>(), 0.0, 0.0);
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

    // Passing mode constants (static values for tower passing)
    public static final double PassHoodDegs = 30.0;
  }

  public final class TurretConstants {
    // Gear tooth counts for dual-encoder absolute angle calculation
    public static final double GEAR_TURRET = 70.0;
    public static final double GEAR_1 = 36.0;
    public static final double GEAR_2 = 34.0;

    public static final double TurretRatio = 1.0;
    public static final InvertedValue Inverted = InvertedValue.Clockwise_Positive;
    public static final double TurretSupplyCurrentLimit = 40.0;

    // Encoder 1
    public static final double TurretEncoderOffsetDegrees = 0.0;
    public static final SensorDirectionValue TurretEncoderDirection =
        SensorDirectionValue.Clockwise_Positive;

    // Encoder 2
    public static final double TurretEncoder2OffsetDegrees = 0.0;
    public static final SensorDirectionValue TurretEncoder2Direction =
        SensorDirectionValue.Clockwise_Positive;

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

    // Manual shooter presets (ABXY)
    public static final double ManualRpsA = 30.0;
    public static final double ManualRpsB = 35.0;
    public static final double ManualRpsX = 40.0;
    public static final double ManualRpsY = 45.0;

    // Passing mode constants (static values for tower passing)
    public static final double PassRps = 40.0;
  }

  public final class FeederConstants {
    // Turntable Constants
    public static final double TurntableRatio =
        1.0 / 1.0; // Sensor rotations to mechanism rotations
    public static final InvertedValue TurntableInverted = InvertedValue.Clockwise_Positive;
    public static final double TurntableSupplyCurrentLimit = 40.0;

    // Turntable motor PID Gains
    public static final double TurntablekP = 0.5;
    public static final double TurntablekI = 0.0;
    public static final double TurntablekD = 0.0;
    public static final double TurntablekV = 0.1;
    public static final double TurntablekS = 0.0;

    public static final double TurntableVelocityToleranceRPS = 0.5;

    // Feed (upward feeding) Constants
    public static final double FeedRatio = 1.0 / 1.0; // Sensor rotations to mechanism rotations
    public static final InvertedValue FeedInverted = InvertedValue.Clockwise_Positive;
    public static final double FeedSupplyCurrentLimit = 40.0;

    // Feed motor PID Gains
    public static final double FeedkP = 0.5;
    public static final double FeedkI = 0.0;
    public static final double FeedkD = 0.0;
    public static final double FeedkV = 0.1;
    public static final double FeedkS = 0.0;

    public static final double FeedVelocityToleranceRPS = 0.5;

    // Default RPS values
    public static final double DefaultTurntableRPS = 5.0;
    public static final double DefaultFeedRPS = 10.0;
  }
}
