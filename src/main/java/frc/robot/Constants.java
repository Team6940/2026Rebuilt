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
    public static final int StretcherMotorID = 23;

    /*   Intake   */
    public static final int IntakeMotorID = 22;

    /*   Shooter   */
    public static final int ShooterMotorID = 33;
    public static final int ShooterFollowerMotorID = 34;

    /*   Hood   */
    public static final int HoodMotorID = 32;

    /*   Turret   */
    public static final int TurretMotorID = 31;
    public static final int TurretEncoderID = 36;
    public static final int TurretEncoder2ID = 35;

    /*   Feeder   */
    public static final int FeederTurntableMotorID = 21;
    public static final int FeederFeedMotorID = 30;

    /*   Climber   */
    public static final int ClimberMotorID = 37;
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

  public final class OperatorConstants {
    /** Joystick deadband applied before scaling operator inputs. */
    public static final double DEADBAND = 0.05;

    /**
     * Power exponent for the input curve. Values > 1 give finer control near center and bolder
     * response near full deflection. 2.0 = quadratic (recommended starting point).
     */
    public static final double INPUT_POWER = 2.0;
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
      InterpolatingDoubleTreeMap shooterRpsZero = new InterpolatingDoubleTreeMap();
      shooterRpsZero.put(0.98, 38.9);
      shooterRpsZero.put(1.61, 41.1);
      shooterRpsZero.put(1.98, 45.2);
      shooterRpsZero.put(2.47, 47.7);
      shooterRpsZero.put(2.90, 50.8);
      shooterRpsZero.put(3.07, 51.27);
      shooterRpsZero.put(3.35, 51.90);
      shooterRpsZero.put(3.53, 54.9);
      shooterRpsZero.put(3.88, 57.8);
      shooterRpsZero.put(4.41, 64.9);
      shooterRpsZero.put(5.20, 70.99);

      // Moving away from target (+1 m/s): needs more spin (+1.5~3 RPS), flatter angle (-0.5~-1 deg)
      InterpolatingDoubleTreeMap shooterRpsPos1 = new InterpolatingDoubleTreeMap();
      shooterRpsPos1.put(1.04, 38.0);
      shooterRpsPos1.put(1.67, 40.2);
      shooterRpsPos1.put(2.05, 44.5);
      shooterRpsPos1.put(2.53, 47.2);
      shooterRpsPos1.put(2.96, 47.7);
      shooterRpsPos1.put(3.41, 50.0);
      shooterRpsPos1.put(3.94, 56.2);
      shooterRpsPos1.put(4.47, 63.4);
      shooterRpsPos1.put(5.26, 69.8);

      // Moving toward target (-1 m/s): needs less spin (-1.5~3 RPS), steeper angle (+0.5~1 deg)
      InterpolatingDoubleTreeMap shooterRpsNeg1 = new InterpolatingDoubleTreeMap();
      shooterRpsNeg1.put(1.04, 35.0);
      shooterRpsNeg1.put(1.67, 36.8);
      shooterRpsNeg1.put(2.05, 41.0);
      shooterRpsNeg1.put(2.53, 43.4);
      shooterRpsNeg1.put(2.96, 43.2);
      shooterRpsNeg1.put(3.41, 45.0);
      shooterRpsNeg1.put(3.94, 50.8);
      shooterRpsNeg1.put(4.47, 57.4);
      shooterRpsNeg1.put(5.26, 62.8);

      RadialVelocityToDistanceToShooterRps.put(-1.0, shooterRpsNeg1);
      RadialVelocityToDistanceToShooterRps.put(0.0, shooterRpsZero);
      RadialVelocityToDistanceToShooterRps.put(1.0, shooterRpsPos1);

      // Radial velocity (m/s) -> Distance (meters) -> Hood position (degrees)

      InterpolatingDoubleTreeMap hoodZero = new InterpolatingDoubleTreeMap();
      hoodZero.put(0.98, 17.5);
      hoodZero.put(1.61, 23.5);
      hoodZero.put(1.99, 28.0);
      hoodZero.put(2.47, 31.2);
      hoodZero.put(2.90, 33.22);
      hoodZero.put(3.35, 35.1);
      hoodZero.put(3.88, 36.1);
      hoodZero.put(4.41, 39.38);
      hoodZero.put(5.20, 41.66);

      // Moving away from target (+1 m/s): more loft needed (+0.5~1 deg)
      InterpolatingDoubleTreeMap hoodPos1 = new InterpolatingDoubleTreeMap();
      hoodPos1.put(1.04, 18.0);
      hoodPos1.put(1.67, 24.2);
      hoodPos1.put(2.05, 28.8);
      hoodPos1.put(2.53, 32.0);
      hoodPos1.put(2.96, 34.0);
      hoodPos1.put(3.41, 36.0);
      hoodPos1.put(3.94, 37.4);
      hoodPos1.put(4.47, 40.3);
      hoodPos1.put(5.26, 42.7);

      // Moving toward target (-1 m/s): less loft needed (-0.5~-1 deg)
      InterpolatingDoubleTreeMap hoodNeg1 = new InterpolatingDoubleTreeMap();
      hoodNeg1.put(1.04, 17.0);
      hoodNeg1.put(1.67, 22.8);
      hoodNeg1.put(2.05, 27.2);
      hoodNeg1.put(2.53, 30.4);
      hoodNeg1.put(2.96, 32.4);
      hoodNeg1.put(3.41, 34.2);
      hoodNeg1.put(3.94, 35.6);
      hoodNeg1.put(4.47, 38.4);
      hoodNeg1.put(5.26, 40.6);

      RadialVelocityToDistanceToHoodPositionDegs.put(-1.0, hoodNeg1);
      RadialVelocityToDistanceToHoodPositionDegs.put(0.0, hoodZero);
      RadialVelocityToDistanceToHoodPositionDegs.put(1.0, hoodPos1);

      // Distance (meters) -> Flight time (seconds)
      DistanceToFlightTimeSecs.put(1.57,0.88);
      DistanceToFlightTimeSecs.put(2.38, 1.11);
      DistanceToFlightTimeSecs.put(2.83, 1.62);
      DistanceToFlightTimeSecs.put(3.81, 1.34);
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
      public static final Translation2d centerPoint = new Translation2d(4.621, 4.07);

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
      public static final Translation2d oppCenterPoint = new Translation2d(11.9, 4.07);
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
    public static final double IntakeRatio = 26. / 12.; // Sensor rotations to mechanism rotations
    public static final InvertedValue IntakeInverted = InvertedValue.Clockwise_Positive;
    public static final double IntakeSupplyCurrentLimit = 40.0;

    // PID Gains
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.18;
    public static final double kS = 0.45;

    public static final double IntakeVelocityToleranceRPS = 0.5;

    public static final double IntakingRPS = 40.0;
    public static final double ReversingRPS = 0;
  }

  public final class HoodConstants {
    public static final double HoodRatio = 182. / 12. * 50. / 8.;
    public static final InvertedValue Inverted = InvertedValue.CounterClockwise_Positive;
    public static final double HoodSupplyCurrentLimit = 40.0;

    // PID Gains
    public static final double kP = 360.;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;
    public static final double kS = 0.29;

    // Motion Magic Gains
    public static final double MaxVelocity = 10.0; // Rotations per second
    public static final double Acceleration = 20.0; // Rotations per second squared

    // Positions (Degrees)
    public static final double HoodPositionToleranceDegs = 1.0;
    public static final double MinDegs = 17.842;
    public static final double MaxDegs = 43.;
    public static final double IdlePosition = 17.842;

    // Manual control tuning
    public static final double HoodManualSensitivity = 2.0;
    public static final double HoodHybridRangeDegs = 2.0;

    // Passing mode constants (static values for tower passing)
    public static final double PassHoodDegs = 30.0;
  }

  public final class TurretConstants {
    // Gear tooth counts for dual-encoder absolute angle calculation
    public static final double GEAR_TURRET = 115.;
    public static final double GEAR_1 = 23.;
    public static final double GEAR_2 = 27.;

    public static final double TurretRatio = 115. / 23. * 48. / 14.;
    public static final InvertedValue Inverted = InvertedValue.CounterClockwise_Positive;
    public static final double TurretSupplyCurrentLimit = 40.0;

    // Encoder 1
    public static final double TurretEncoderOffsetDegrees = -228.779;
    public static final SensorDirectionValue TurretEncoderDirection =
        SensorDirectionValue.CounterClockwise_Positive;

    // Encoder 2
    public static final double TurretEncoder2OffsetDegrees = -323.613;
    public static final SensorDirectionValue TurretEncoder2Direction =
        SensorDirectionValue.CounterClockwise_Positive;

    // PID Gains
    public static final double kP = 42.;
    public static final double kI = 0.0;
    public static final double kD = 1.2;
    public static final double kV = 0.0;
    public static final double kS = 0.35;

    // Motion Magic Gains
    public static final double MaxVelocity = 40.0; // Rotations per second
    public static final double Acceleration = 60.0; // Rotations per second squared

    // Positions (Degrees)
    public static final double TurretPositionToleranceDegs = 2.0;
    public static final double MinDegs = -220.0;
    public static final double MaxDegs = 225.0;
    public static final double IdlePosition = 20.0;

    // Manual control tuning
    public static final double TurretManualSensitivity = 5.0;
    public static final double TurretHybridRangeDegs = 15.0;

    /**
     * Turret pivot offset from the chassis center, in robot frame (+X = forward, +Y = left).
     * The turret is mounted behind the chassis center, hence negative X.
     * Tune the X value to the actual measured distance (meters).
     */
    public static final Translation2d TURRET_OFFSET = new Translation2d(-0.05, 0.0);
  }

  public final class StretcherConstants {
    public static final double StretcherRatio =
        27. * 26. / 46.; // Sensor rotations to mechanism rotations
    public static final InvertedValue Inverted = InvertedValue.CounterClockwise_Positive;
    public static final double StretcherSupplyCurrentLimit = 40.0;

    // PID Gains
    public static final double kP = 96.;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;
    public static final double kS = 0.34;
    // public static final double kG = 0.3;

    // Motion Magic Gains
    public static final double MaxVelocity = 20.0; // Rotations per second
    public static final double Acceleration = 48.0; // Rotations per second squared

    // Positions (Rotations) — original degree values divided by 360
    public static final double StretcherPositionToleranceRotations = 3.0 / 360.0;

    public static final double MinRotations = 0.; // CCW Positive
    public static final double MaxRotations = 3.11;

    public static final double ExtendedPosition = 2.65;
    public static final double RetractedPosition = 0.08;

    public static final double IdlePosition = 0.0;
  }

  public final class ShooterConstants {
    public static final double ShooterRatio = 1.0 / 1.0;
    public static final InvertedValue Inverted = InvertedValue.Clockwise_Positive;
    public static final double ShooterVelocityToleranceRPS = 0.5;
    public static final MotorAlignmentValue FollowerAlignment = MotorAlignmentValue.Aligned;

    // PID Gains
    public static final double kP = 11.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.115;
    public static final double kS = 21.;

    // Torque-current feedforward (Amps per RPS)
    public static final double kTorqueFFPerRPS = 0.0;

    // Consider using a static Torque feedforward here
    public static final double kT = 0.1;

    // Current limits
    public static final double ShooterSupplyCurrentLimit = 80.0;

    // Manual shooter presets (ABXY)
    public static final double ManualRpsA = 20.;
    public static final double ManualRpsB = 40.;
    public static final double ManualRpsX = 55.;
    public static final double ManualRpsY = 70.;

    // Passing mode constants (static values for tower passing)
    public static final double PassRps = 40.0;
  }

  public final class FeederConstants {
    // Turntable Constants
    public static final double TurntableRatio = 31.2; // Sensor rotations to mechanism rotations
    public static final InvertedValue TurntableInverted = InvertedValue.CounterClockwise_Positive;
    public static final double TurntableSupplyCurrentLimit = 40.0;

    // Turntable motor PID Gains
    public static final double TurntablekP = 0.4;
    public static final double TurntablekI = 0.0;
    public static final double TurntablekD = 0.0;
    public static final double TurntablekV = 4.;
    public static final double TurntablekS = 0.392;

    public static final double TurntableVelocityToleranceRPS = 0.5;

    // Feed (upward feeding) Constants
    public static final double FeedRatio = 1.0 / 1.0; // Sensor rotations to mechanism rotations
    public static final InvertedValue FeedInverted = InvertedValue.CounterClockwise_Positive;
    public static final double FeedSupplyCurrentLimit = 40.0;

    // Feed motor PID Gains
    public static final double FeedkP = 0.15;
    public static final double FeedkI = 0.0;
    public static final double FeedkD = 0.0;
    public static final double FeedkV = 0.105;
    public static final double FeedkS = 0.452;

    public static final double FeedVelocityToleranceRPS = 0.5;

    // Default RPS values
    public static final double DefaultTurntableRPS = 2.3;
    public static final double DefaultFeedRPS = 100.0;
  }

  public final class ClimberConstants {
    public static final double ClimberRatio = 45.; // Sensor rotations to mechanism rotations
    public static final InvertedValue Inverted = InvertedValue.Clockwise_Positive;
    public static final double ClimberSupplyCurrentLimit = 60.0;

    // PID Gains
    public static final double kP = 40.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;
    public static final double kS = 0.2;

    // Motion Magic Gains
    public static final double MaxVelocity = 16.; // Rotations per second
    public static final double Acceleration = 32.; // Rotations per second squared

    // Positions (Rotations)
    public static final double ClimberPositionToleranceRotations = 5.0 / 360.0;
    public static final double MinRotations = -0.2;
    public static final double MaxRotations = 1.8; // 180 degrees = 0.5 rotations
    public static final double IdlePosition = 0.0;

    // Preset positions (Rotations)
    public static final double RetractedPosition = 0.0;
    public static final double ExtendedPosition = 1.8; // 90 degrees = 0.25 rotations
  }

  public static final class PoseEstimatorConstants {
    public static final InterpolatingDoubleTreeMap tAtoDev = new InterpolatingDoubleTreeMap();

    static {
      tAtoDev.put(0.17, 0.08);
      tAtoDev.put(0.12, 0.20);
      tAtoDev.put(0.071, 0.35);
      tAtoDev.put(0.046, 0.4);
    }
  }
}
