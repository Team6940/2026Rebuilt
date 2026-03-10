package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;
import frc.robot.util.SetpointLeadCompensator;
import frc.robot.util.TurretVelocityCalculator;
import java.util.function.DoubleSupplier;
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
  private double rawSetpointDegs = TurretConstants.IdlePosition;
  private double TargetVelocity=0;
  private double operatorInputScalar = 0.0;
  private double encoderCalculatedPositionDegs = TurretConstants.IdlePosition;

  // Lead compensator: applied to the final setpoint regardless of mode
  private static final double TURRET_LEAD_INDEX = 0.08; // seconds; tune this
  private final SetpointLeadCompensator leadComp = new SetpointLeadCompensator(TURRET_LEAD_INDEX);

  private Rotation2d fieldRelativeRotation2d = new Rotation2d();
  private Pose2d lastRobotPose = new Pose2d();

  // Velocity feedforward calculator: trapezoidal profile + chassis omega compensation
  private final TurretVelocityCalculator velocityCalc =
      new TurretVelocityCalculator(
          TurretConstants.ProfileMaxVelocityDegsPerSec, TurretConstants.ProfileMaxAccelDegsPerSec2);
  private DoubleSupplier chassisOmegaRadPerSecSupplier = () -> 0.0;
  private double velocityFFDegsPerSec = 0.0;

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

  /** Explicit alias for {@link #setPosition(double, double)} with a descriptive name. */
  public void setPositionWithVelocity(double positionDegrees, double velocityDegsPerSec) {
    positionDegrees = clamp(positionDegrees);
    io.setPositionWithVelocity(positionDegrees, velocityDegsPerSec);
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
   * Provides the chassis angular velocity to the turret so it can compensate for robot rotation.
   * Call this once (e.g. in command initialize) to wire in the supplier; it will be read every
   * loop.
   *
   * @param omegaSupplier supplies chassis omega in radians/second (CCW-positive)
   */
  public void setChassisOmegaSupplier(DoubleSupplier omegaSupplier) {
    chassisOmegaRadPerSecSupplier = omegaSupplier;
  }

  public TurretMode getMode() {
    return mode;
  }

  public void setAutoSetpoint(double positionDegrees) {
    autoSetpointDegs = clamp(positionDegrees);
  }

  public void setAutoSetpoint(double positionDegrees,double targetVelocity) {
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
    Logger.recordOutput("Turret/VelocityFFDegsPerSec", velocityFFDegsPerSec);
    Logger.recordOutput("Turret/ProfiledPositionDegs", velocityCalc.getProfiledPosition());
    Logger.recordOutput("Turret/ProfiledVelocityDegsPerSec", velocityCalc.getProfiledVelocity());
    Logger.recordOutput(
        "Turret/ChassisOmegaCompDegsPerSec",
        -Math.toDegrees(chassisOmegaRadPerSecSupplier.getAsDouble()));
  }

  private static final double LOOP_DT = 0.02;

  private void handleHybrid() {
    rawSetpointDegs =
        clamp(autoSetpointDegs + operatorInputScalar * TurretConstants.TurretHybridRangeDegs);
    // targetPositionDegs = rawSetpointDegs;
    // TrapezoidProfile.State targetState=
    //     velocityCalc.calculate(
    //         targetPositionDegs, chassisOmegaRadPerSecSupplier.getAsDouble(), LOOP_DT);
    // targetPositionDegs=targetState.position;
    // velocityFFDegsPerSec=targetState.velocity;
    setPositionWithVelocity(rawSetpointDegs, velocityFFDegsPerSec);
  }

  private void handleManual() {
    nudgeManualSetpoint(operatorInputScalar);
    rawSetpointDegs = clamp(manualSetpointDegs);
    //  TrapezoidProfile.State targetState=
    //     velocityCalc.calculate(
    //         targetPositionDegs, chassisOmegaRadPerSecSupplier.getAsDouble(), LOOP_DT);
    // targetPositionDegs=targetState.position;
    // velocityFFDegsPerSec=targetState.velocity;
    setPositionWithVelocity(rawSetpointDegs, velocityFFDegsPerSec);
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
