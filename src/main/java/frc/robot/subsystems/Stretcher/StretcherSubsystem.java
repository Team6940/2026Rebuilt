package frc.robot.subsystems.Stretcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StretcherConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class StretcherSubsystem extends SubsystemBase {
  public static StretcherSubsystem m_instance = null;

  public static StretcherSubsystem getInstance() {
    return m_instance == null ? m_instance = new StretcherSubsystem() : m_instance;
  }

  private final StretcherIO io;
  private final StretcherIOInputsAutoLogged inputs = new StretcherIOInputsAutoLogged();

  /** Current commanded target position in rotations. */
  private double targetPositionRotations = StretcherConstants.IdlePosition;

  public StretcherSubsystem() {
    if (Robot.isReal()) {
      io = new StretcherIOPhoenix6();
    } else {
      // TODO: Implement simulation code here
      io = new StretcherIO() {};
    }
  }

  // ── Public API ────────────────────────────────────────────────────────────

  /**
   * Commands the stretcher to a target position.
   *
   * @param positionRotations Target arm position in rotations (clamped to [MinRotations,
   *     MaxRotations]).
   */
  public void setPosition(double positionRotations) {
    targetPositionRotations =
        MathUtil.clamp(
            positionRotations, StretcherConstants.MinRotations, StretcherConstants.MaxRotations);
    io.setPosition(targetPositionRotations);
  }

  /** Extends the stretcher to the intake position. */
  public void extend() {
    setPosition(StretcherConstants.ExtendedPosition);
  }

  /** Retracts the stretcher to the stowed position. */
  public void retract() {
    setPosition(StretcherConstants.RetractedPosition);
  }

  /** Commands the stretcher to the idle (zero) position. */
  public void goIdle() {
    setPosition(StretcherConstants.IdlePosition);
  }

  public void zeroStretcherPosition() {
    io.zeroStretcherPosition();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setCoast() {
    io.setCoast();
  }

  /** Returns the current measured arm position in rotations. */
  public double getCurrentPositionRotations() {
    return inputs.stretcherPositionRotations;
  }

  /** Returns the current target position in rotations. */
  public double getTargetPositionRotations() {
    return targetPositionRotations;
  }

  /** Returns true when the arm is within tolerance of the target position. */
  public boolean isAtTargetPosition() {
    return MathUtil.isNear(
        targetPositionRotations,
        inputs.stretcherPositionRotations,
        StretcherConstants.StretcherPositionToleranceRotations);
  }

  public double getPosition() {
    return inputs.stretcherPositionRotations;
  }

  // ── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Stretcher", inputs);
    Logger.recordOutput("Stretcher/TargetPositionRotations", targetPositionRotations);
    Logger.recordOutput("Stretcher/ActualPositionRotations", inputs.stretcherPositionRotations);
    Logger.recordOutput("Stretcher/IsAtTarget", isAtTargetPosition());
    Logger.recordOutput("Stretcher/MotorConnected", inputs.motorConnected);
  }
}
