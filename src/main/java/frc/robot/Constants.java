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

import edu.wpi.first.wpilibj.RobotBase;

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

    /*   Intaker   */
    public static final int IntakerMotorID = 15;


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
  }

  public final class IntakerConstants {
    public static final double IntakerRatio = 1.0 / 1.0; // Sensor rotations to mechanism rotations
    public static final InvertedValue IntakerInverted = InvertedValue.Clockwise_Positive;

    // PID Gains
    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.1;
    public static final double kS = 0.0;

    public static final double IntakerVelocityToleranceRPS = 0.5;

    public static final double IntakingRPS = 10.0;
    public static final double ReversingRPS = 0;

  }

  public final class StretcherConstants {
    public static final double StretcherRatio = 1.0 / 1.0; // Sensor rotations to mechanism rotations
    public static final InvertedValue Inverted = InvertedValue.Clockwise_Positive;
    public static final double StretcherVelocityToleranceRPS = 0.2;

    // PID Gains
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.2;
    public static final double kS = 0.0;
    public static final double kG = 0.3;

    // Motion Magic Gains
    public static final double MaxVelocity = 4.0; // Rotations per second
    public static final double Acceleration = 8.0; // Rotations per second squared
    //public static final double Deadband = 0.24;

    // Positions
    public static final double StretcherPositionToleranceDegs = 3.;

    public static final double MinDegs = -66.; //degrees CCW Positive
    public static final double MaxDegs = 90.; 

    public static final double ExtendedPosition = -61.;
    public static final double RetractedPosition = 90.;
  }
}
