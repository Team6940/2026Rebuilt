package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  public static TurretSubsystem m_instance;

  public static TurretSubsystem getInstance() {
    return m_instance == null ? m_instance = new TurretSubsystem() : m_instance;
  }

  public enum TurretMode {
    HYBRID,
    MANUAL
  }

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private TurretMode mode = TurretMode.HYBRID;
  private double autoSetpointDegs = TurretConstants.IdlePosition;
  private double manualSetpointDegs = TurretConstants.IdlePosition;
  private double targetPositionDegs = TurretConstants.IdlePosition;
  private double operatorInputScalar = 0.0;
  private double encoderCalculatedPositionDegs = TurretConstants.IdlePosition;

  private Rotation2d fieldRelativeRotation2d = new Rotation2d();
  private Pose2d lastRobotPose = new Pose2d();

  public TurretSubsystem() {
    if (Robot.isReal()) {
      io = new TurretIOPhoenix6();
    } else {
      // TODO: Implement simulation code here
      io = new TurretIO() {};
    }
    io.updateInputs(inputs); // this is necessary to initialize the inputs
    encoderCalculatedPositionDegs =
        calculateTurretDegsFromEncoders(
            inputs.encoderPositionDegrees, inputs.encoder2PositionDegrees);
    resetPosition(encoderCalculatedPositionDegs);
    // this is added to ensure the turret starts at the correct position
  }

  public void resetPosition(double positionDegrees) {
    io.resetPosition(positionDegrees);
  }

  public void setPosition(double positionDegrees) {
    positionDegrees = clamp(positionDegrees);
    io.setPosition(positionDegrees);
  }

  public void setModeHybrid() {
    mode = TurretMode.HYBRID;
    autoSetpointDegs = clamp(autoSetpointDegs);
  }

  public void setModeManual() {
    mode = TurretMode.MANUAL;
    manualSetpointDegs = clamp(manualSetpointDegs);
  }

  public TurretMode getMode() {
    return mode;
  }

  public void setAutoSetpoint(double positionDegrees) {
    autoSetpointDegs = clamp(positionDegrees);
  }

  /*
   * Sets the turret to a field-relative angle by calculating the
   * turret-relative angle based on the robot's current pose.
   * Automatically finds the nearest equivalent angle within the turret's range.
   */
  public void setAutoSetpointFieldRelativeRotation2d(Rotation2d fieldAngle, Pose2d robotPose) {
    fieldRelativeRotation2d = fieldAngle;
    lastRobotPose = robotPose;
    Rotation2d turretRelative = fieldAngle.minus(robotPose.getRotation());

    // Find the nearest equivalent angle within the turret's range
    double targetAngle =
        findNearestEquivalentAngle(
            turretRelative.getDegrees(),
            inputs.turretPositionDegrees,
            TurretConstants.MinDegs,
            TurretConstants.MaxDegs);

    Logger.recordOutput("Turret/FieldRelativeRotation2d", fieldRelativeRotation2d.getDegrees());
    Logger.recordOutput("Turret/RawTurretRelativeDegs", turretRelative.getDegrees());
    Logger.recordOutput("Turret/ChosenEquivalentDegs", targetAngle);
    setAutoSetpoint(targetAngle);
  }

  /**
   * Finds the nearest equivalent angle to the current position within the given range. For example,
   * if current position is 240°, target is 0°, min is -360°, max is 360°, it will choose 360°
   * (equivalent to 0°) as it's closer than 0° or -360°.
   */
  private double findNearestEquivalentAngle(
      double targetDeg, double currentDeg, double minDeg, double maxDeg) {
    double bestAngle = targetDeg;
    double minDistance = Double.MAX_VALUE;

    // Normalize target to [-180, 180) range first
    targetDeg = normalizeAngle(targetDeg);

    // Check all equivalent angles by adding/subtracting 360° multiples
    // We need to check enough rotations to cover the entire range
    int maxRotations = (int) Math.ceil((maxDeg - minDeg) / 360.0) + 1;

    for (int i = -maxRotations; i <= maxRotations; i++) {
      double candidate = targetDeg + (i * 360.0);

      // Check if this candidate is within the valid range
      if (candidate >= minDeg && candidate <= maxDeg) {
        double distance = Math.abs(candidate - currentDeg);

        if (distance < minDistance) {
          minDistance = distance;
          bestAngle = candidate;
        }
      }
    }

    return bestAngle;
  }

  /** Normalizes an angle to the range [-180, 180) degrees. */
  private double normalizeAngle(double degrees) {
    double angle = degrees % 360.0;
    if (angle >= 180.0) {
      angle -= 360.0;
    } else if (angle < -180.0) {
      angle += 360.0;
    }
    return angle;
  }

  public void setManualSetpoint(double positionDegrees) {
    manualSetpointDegs = clamp(positionDegrees);
  }

  public void setOperatorInputScalar(double scalar) {
    operatorInputScalar = MathUtil.clamp(scalar, -1.0, 1.0);
  }

  public void nudgeManualSetpoint(double scalar) {
    manualSetpointDegs =
        clamp(manualSetpointDegs + scalar * TurretConstants.TurretManualSensitivity);
  }

  public double getTargetPositionDegs() {
    return targetPositionDegs;
  }

  public Rotation2d getTurretFieldAngle(Pose2d robotPose) {
    return robotPose.getRotation().plus(Rotation2d.fromDegrees(inputs.turretPositionDegrees));
  }

  public boolean isAtTargetPosition() {
    return MathUtil.isNear(
        targetPositionDegs,
        inputs.turretPositionDegrees,
        TurretConstants.TurretPositionToleranceDegs);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (inputs.encoderConnected && inputs.encoder2Connected) {
      encoderCalculatedPositionDegs =
          calculateTurretDegsFromEncoders(
              inputs.encoderPositionDegrees, inputs.encoder2PositionDegrees);
    }

    switch (mode) {
      case HYBRID -> handleHybrid();
      case MANUAL -> handleManual();
    }

    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/Mode", mode.toString());
    Logger.recordOutput("Turret/TargetPositionDegs", targetPositionDegs);
    Logger.recordOutput("Turret/IsAtTarget", isAtTargetPosition());
    Logger.recordOutput("Turret/AutoSetpointDegs", autoSetpointDegs);
    Logger.recordOutput("Turret/ManualSetpointDegs", manualSetpointDegs);
    Logger.recordOutput("Turret/FieldTargetDegs", fieldRelativeRotation2d.getDegrees());
    Logger.recordOutput("Turret/RobotHeadingDegs", lastRobotPose.getRotation().getDegrees());
    Logger.recordOutput("Turret/OperatorInputScalar", operatorInputScalar);
    Logger.recordOutput("Turret/EncoderCalculatedPositionDegs", encoderCalculatedPositionDegs);
  }

  private void handleHybrid() {
    targetPositionDegs =
        clamp(autoSetpointDegs + operatorInputScalar * TurretConstants.TurretHybridRangeDegs);
    io.setPosition(targetPositionDegs);
  }

  private void handleManual() {
    nudgeManualSetpoint(operatorInputScalar);
    targetPositionDegs = clamp(manualSetpointDegs);
    io.setPosition(targetPositionDegs);
  }

  private double clamp(double positionDegrees) {
    return MathUtil.clamp(positionDegrees, TurretConstants.MinDegs, TurretConstants.MaxDegs);
  }

  private double calculateTurretDegsFromEncoders(double encoder1Deg, double encoder2Deg) {
    double difference = encoder2Deg - encoder1Deg;
    if (difference > 250.0) {
      difference -= 360.0;
    }
    if (difference < -250.0) {
      difference += 360.0;
    }

    double slope =
        (TurretConstants.GEAR_2 * TurretConstants.GEAR_1)
            / ((TurretConstants.GEAR_1 - TurretConstants.GEAR_2) * TurretConstants.GEAR_TURRET);
    difference *= slope;

    double encoder1Rotations =
        (difference * TurretConstants.GEAR_TURRET / TurretConstants.GEAR_1) / 360.0;
    double encoder1RotationsFloored = Math.floor(encoder1Rotations);
    double turretAngle =
        (encoder1RotationsFloored * 360.0 + encoder1Deg)
            * (TurretConstants.GEAR_1 / TurretConstants.GEAR_TURRET);

    if (turretAngle - difference < -100.0) {
      turretAngle += (TurretConstants.GEAR_1 / TurretConstants.GEAR_TURRET) * 360.0;
    } else if (turretAngle - difference > 100.0) {
      turretAngle -= (TurretConstants.GEAR_1 / TurretConstants.GEAR_TURRET) * 360.0;
    }

    return turretAngle;
  }
}
