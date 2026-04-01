package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;
import frc.robot.util.SetpointLeadCompensator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  public static TurretSubsystem m_instance;

  public static TurretSubsystem getInstance() {
    return m_instance == null ? m_instance = new TurretSubsystem() : m_instance;
  }

  public enum TurretMode {
    HYBRID,
    MANUAL,
    VELOCITY
  }

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private TurretMode mode = TurretMode.HYBRID;
  private double autoSetpointDegs = TurretConstants.IdlePosition;
  private double manualSetpointDegs = TurretConstants.IdlePosition;
  private double targetPositionDegs = TurretConstants.IdlePosition;
  private double rawSetpointDegs = TurretConstants.IdlePosition;
  private double operatorInputScalar = 0.0;
  private double encoderCalculatedPositionDegs = TurretConstants.IdlePosition;

  // Lead compensator: applied to the final setpoint regardless of mode
  private static final double TURRET_LEAD_INDEX = 0.08; // seconds; tune this
  private final SetpointLeadCompensator leadComp = new SetpointLeadCompensator(TURRET_LEAD_INDEX);

  private Rotation2d fieldRelativeRotation2d = new Rotation2d();
  private Pose2d lastRobotPose = new Pose2d();

  private DoubleSupplier robotOmegaRadPerSecSupplier = () -> 0.0;
  private DoubleSupplier targetVelFFDegsPerSecSupplier = () -> 0.0;

  // Diagnostic log fields (written in handleVelocity, read in periodic)
  private double velocityCmdDegsPerSec = 0.0;
  private double dbg_positionError = 0.0;
  private double dbg_targetVelFF = 0.0;
  private double dbg_chassisFF = 0.0;

  public TurretSubsystem() {
    if (Robot.isReal()) {
      io = new TurretIOPhoenix6();
    } else {
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

  public void setVelocity(double velocityDegsPerSec) {
    io.setVelocity(
        MathUtil.clamp(
            velocityDegsPerSec,
            -TurretConstants.VelocityModeMaxDegsPerSec,
            TurretConstants.VelocityModeMaxDegsPerSec));
  }

  public void setModeHybrid() {
    mode = TurretMode.HYBRID;
    autoSetpointDegs = clamp(autoSetpointDegs);
  }

  public void setModeManual() {
    mode = TurretMode.MANUAL;
    manualSetpointDegs = clamp(manualSetpointDegs);
  }

  /**
   * Enters VELOCITY mode.
   *
   * @param omegaRadPerSecSupplier chassis yaw rate (rad/s, CCW-positive) for field-relative FF
   * @param targetVelFFDegsPerSecSupplier commanded angular velocity feedforward (deg/s) derived
   *     externally from the target's tangential motion; pass {@code () -> 0.0} when not needed
   */
  public void setModeVelocity(
      DoubleSupplier omegaRadPerSecSupplier, DoubleSupplier targetVelFFDegsPerSecSupplier) {
    mode = TurretMode.VELOCITY;
    robotOmegaRadPerSecSupplier = omegaRadPerSecSupplier;
    this.targetVelFFDegsPerSecSupplier = targetVelFFDegsPerSecSupplier;
  }

  public TurretMode getMode() {
    return mode;
  }

  public void setAutoSetpoint(double positionDegrees) {
    autoSetpointDegs = clamp(positionDegrees);
  }

  public void setAutoSetpoint(double positionDegrees, double targetVelocity) {
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
  public double findNearestEquivalentAngle(
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

  public double getManualSetpointDegs() {
    return manualSetpointDegs;
  }

  public double getCurrentPositionDegs() {
    return inputs.turretPositionDegrees;
  }

  public double getTargetPositionDegs() {
    return targetPositionDegs;
  }

  public Rotation2d getTurretFieldAngle(Pose2d robotPose) {
    return robotPose.getRotation().plus(Rotation2d.fromDegrees(inputs.turretPositionDegrees));
  }

  //   /**
  //  * Computes the turret angle that would point to the desired field-relative angle,
  //  * based on the robot's current pose and turret position.
  //  * Feat with a margin of to prevent jittering when the target is near the edge of the turret's
  // range.
  //  *
  //  * @param desiredFieldAngle the field-relative angle we want the turret to point to
  //  * @param robotPose
  //  */
  // public double getTurretFieldAngle(
  //     Rotation2d desiredFieldAngle,
  //     Pose2d robotPose) {
  //     double currentTurretDeg = inputs.turretPositionDegrees;
  //     Rotation2d currentFieldAngle = getTurretFieldAngle(robotPose);
  //     double fieldError = desiredFieldAngle.minus(currentFieldAngle).getDegrees();
  //     double deltaSmall = normalizeAngle(fieldError);
  //     double candidateSmall = currentTurretDeg + deltaSmall;
  //     double margin = 1.0; // 1° margin
  //     double limitMin = -180.0 + margin;
  //     double limitMax = 180.0 - margin;
  //     if (candidateSmall >= limitMin && candidateSmall <= limitMax) {
  //         return currentTurretDeg + deltaSmall;
  //     }
  //     double deltaLarge = (deltaSmall > 0) ? deltaSmall - 360.0 : deltaSmall + 360.0;
  //     double candidateLarge = currentTurretDeg + deltaLarge;
  //     if (candidateLarge >= limitMin && candidateLarge <= limitMax) {
  //         return currentTurretDeg + deltaLarge;
  //     }
  //     return currentTurretDeg;
  // }

  public boolean isAtTargetPosition() {
    return MathUtil.isNear(
        autoSetpointDegs,
        inputs.turretPositionDegrees,
        TurretConstants.TurretPositionToleranceDegs);
  }

  public boolean isAtTargetPosition(double distance) {
    return MathUtil.isNear(
        autoSetpointDegs,
        inputs.turretPositionDegrees,
        TurretConstants.DistanceToTurretTolerance.get(distance));
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
      case VELOCITY -> handleVelocity();
        // case MANUAL -> handleManualFieldRelative();
    }

    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/Mode", mode.toString());
    Logger.recordOutput("Turret/TargetPositionDegs", targetPositionDegs);
    Logger.recordOutput("Turret/RawSetpointDegs", rawSetpointDegs);
    Logger.recordOutput("Turret/IsAtTarget", isAtTargetPosition());
    Logger.recordOutput("Turret/AutoSetpointDegs", autoSetpointDegs);
    Logger.recordOutput("Turret/ManualSetpointDegs", manualSetpointDegs);
    Logger.recordOutput("Turret/LeadDerivativeDegPerSec", leadComp.getFilteredDerivative());
    Logger.recordOutput("Turret/LeadCompDeltaDegs", targetPositionDegs - rawSetpointDegs);
    Logger.recordOutput("Turret/FieldTargetDegs", fieldRelativeRotation2d.getDegrees());
    Logger.recordOutput("Turret/RobotHeadingDegs", lastRobotPose.getRotation().getDegrees());
    Logger.recordOutput("Turret/OperatorInputScalar", operatorInputScalar);
    Logger.recordOutput("Turret/EncoderCalculatedPositionDegs", encoderCalculatedPositionDegs);
    Logger.recordOutput("Turret/VelocityCmdDegsPerSec", velocityCmdDegsPerSec);
    Logger.recordOutput("Turret/MotorVelocityDegsPerSec", inputs.motorVelocityDegsPerSec);
    Logger.recordOutput("Turret/Velocity/PositionError", dbg_positionError);
    Logger.recordOutput("Turret/Velocity/TargetVelFF", dbg_targetVelFF);
    Logger.recordOutput("Turret/Velocity/ChassisFF", dbg_chassisFF);
  }

  private void handleHybrid() {
    rawSetpointDegs =
        clamp(autoSetpointDegs + operatorInputScalar * TurretConstants.TurretHybridRangeDegs);
    targetPositionDegs = rawSetpointDegs;
    // TrapezoidProfile.State targetState=
    //     velocityCalc.calculate(
    //         targetPositionDegs, chassisOmegaRadPerSecSupplier.getAsDouble(), LOOP_DT);
    // targetPositionDegs=targetState.position;
    // velocityFFDegsPerSec=targetState.velocity;
    setPosition(targetPositionDegs);
  }

  private void handleManual() {
    nudgeManualSetpoint(operatorInputScalar);
    rawSetpointDegs = clamp(manualSetpointDegs);
    targetPositionDegs = rawSetpointDegs;
    //  TrapezoidProfile.State targetState=
    //     velocityCalc.calculate(
    //         targetPositionDegs, chassisOmegaRadPerSecSupplier.getAsDouble(), LOOP_DT);
    // targetPositionDegs=targetState.position;
    // velocityFFDegsPerSec=targetState.velocity;
    setPosition(targetPositionDegs);
  }

  private static final double VELOCITY_LOOP_DT = 0.02; // seconds (standard 50 Hz loop)
  // Deceleration zone: start ramping down velocity this many degrees before the hard limit
  private static final double LIMIT_DECEL_ZONE_DEGS = 10.0;

  // Note: mapping of external setpoints into the turret's mechanical range is
  // handled by findNearestEquivalentAngle(..). The previous wrapToValidRange
  // helper was removed to avoid duplication.

  private void handleVelocity() {
    // ── Step 1: position error ────────────────────────────────────────────────
    rawSetpointDegs =
        clamp(autoSetpointDegs + operatorInputScalar * TurretConstants.TurretHybridRangeDegs);
    targetPositionDegs = rawSetpointDegs;
    double positionError = targetPositionDegs - inputs.turretPositionDegrees;
    double pTerm = TurretConstants.kP_position * positionError;

    // ── Step 2: feedforward terms ─────────────────────────────────────────────
    // Target velocity FF — supplied externally by the command, which computes it as
    //   tangentialVelocityToVirtualTarget / distanceToVirtualTarget  (rad/s → deg/s).
    // Scaled by kFF_targetVel for independent gain tuning.
    double targetVelFF =
        TurretConstants.kFF_targetVel * targetVelFFDegsPerSecSupplier.getAsDouble();

    // Chassis yaw FF — keeps the turret field-relative while the robot rotates.
    // Robot CCW positive → turret must rotate CW → negative sign.
    // Scaled by kFF_chassis so it can be dialled down independently.
    double chassisFF =
        TurretConstants.kFF_chassis * (-Math.toDegrees(robotOmegaRadPerSecSupplier.getAsDouble()));

    // ── Step 3: sum and clamp ─────────────────────────────────────────────────
    double rawCmd = pTerm + targetVelFF + chassisFF;
    rawCmd =
        MathUtil.clamp(
            rawCmd,
            -TurretConstants.VelocityModeMaxDegsPerSec,
            TurretConstants.VelocityModeMaxDegsPerSec);

    // ── Step 4: soft deceleration zone near mechanical limits ─────────────────
    double pos = inputs.turretPositionDegrees;
    if (rawCmd > 0) {
      double distToMax = TurretConstants.MaxDegs - pos;
      if (distToMax <= 0) {
        rawCmd = 0;
      } else if (distToMax < LIMIT_DECEL_ZONE_DEGS) {
        rawCmd *= distToMax / LIMIT_DECEL_ZONE_DEGS;
        if (pos + rawCmd * VELOCITY_LOOP_DT > TurretConstants.MaxDegs) rawCmd = 0;
      }
    } else if (rawCmd < 0) {
      double distToMin = pos - TurretConstants.MinDegs;
      if (distToMin <= 0) {
        rawCmd = 0;
      } else if (distToMin < LIMIT_DECEL_ZONE_DEGS) {
        rawCmd *= distToMin / LIMIT_DECEL_ZONE_DEGS;
        if (pos + rawCmd * VELOCITY_LOOP_DT < TurretConstants.MinDegs) rawCmd = 0;
      }
    }

    // ── Step 5: apply ─────────────────────────────────────────────────────────
    velocityCmdDegsPerSec = rawCmd;
    dbg_positionError = positionError;
    dbg_targetVelFF = targetVelFF;
    dbg_chassisFF = chassisFF;
    setVelocity(velocityCmdDegsPerSec);
  }

  // private void handleManualFieldRelative() {
  //   targetPositionDegs = manualSetpointDegs;
  //   io.setPosition(targetPositionDegs);
  // }

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
