package frc.robot.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.TurretConstants;

/**
 * Computes the feedforward velocity (deg/s) that the turret motor should track in order to follow a
 * changing position setpoint without lag.
 *
 * <p>Two components are summed:
 *
 * <ol>
 *   <li><b>Trapezoidal profile velocity</b> — the velocity the turret-relative setpoint demands in
 *       order to reach the goal along a trapezoidal motion profile (smooth acceleration /
 *       deceleration). This captures the "derivative of the setpoint" in a well-behaved way.
 *   <li><b>Chassis omega compensation</b> — when the robot rotates, the turret must counter-rotate
 *       at the same rate to maintain a field-fixed aim direction. This is injected as {@code
 *       −ω_chassis} converted to turret deg/s.
 * </ol>
 *
 * The combined velocity is passed to {@code PositionTorqueCurrentFOC.withVelocity()} so the
 * motor's internal PID receives an accurate velocity feedforward and does not lag behind.
 */
public class TurretVelocityCalculator {

  private final TrapezoidProfile profile;
  private TrapezoidProfile.State currentState;
  private TrapezoidProfile.State lastState;

  /**
   * @param maxVelocityDegsPerSec maximum turret velocity in degrees/second
   * @param maxAccelerationDegsPerSec2 maximum turret acceleration in degrees/second²
   */
  public TurretVelocityCalculator(
      double maxVelocityDegsPerSec, double maxAccelerationDegsPerSec2) {
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocityDegsPerSec, maxAccelerationDegsPerSec2));
    currentState = new TrapezoidProfile.State(TurretConstants.IdlePosition, 0.0);
    lastState=currentState;
  }

  /**
   * Call once per loop iteration. Returns the profile velocity (deg/s) that the turret should be
   * travelling at to smoothly reach {@code goalPositionDegs}.
   *
   * @param goalPositionDegs the desired turret position (degrees)
   * @param dtSeconds loop period (seconds), typically 0.02
   * @return profile velocity in degrees/second (setpoint derivative only, before chassis omega)
   */
  public TrapezoidProfile.State calculate(double goalPositionDegs, double dtSeconds) {
    TrapezoidProfile.State goal = new TrapezoidProfile.State(goalPositionDegs, 0.0);
    lastState=currentState;
    currentState = profile.calculate(dtSeconds, lastState, goal);

    return currentState;
  }
  

  /**
   * Convenience method that also folds in chassis omega compensation.
   *
   * @param goalPositionDegs the desired turret position (degrees)
   * @param chassisOmegaRadPerSec robot angular velocity (rad/s, CCW-positive)
   * @param dtSeconds loop period
   * @return total velocity feedforward in degrees/second
   */
  public TrapezoidProfile.State calculate(
      double goalPositionDegs, double chassisOmegaRadPerSec, double dtSeconds) {
    TrapezoidProfile.State profileState=calculate(goalPositionDegs, dtSeconds);
    double profileVel = profileState.velocity;
    double profilePosition=profileState.position;
    double omegaCompensation = -Math.toDegrees(chassisOmegaRadPerSec);
    return new TrapezoidProfile.State(profilePosition, profileVel + omegaCompensation);
  }

  /** Returns the current profiled position (deg) — can be used for logging. */
  public double getProfiledPosition() {
    return currentState.position;
  }

  /** Returns the current profiled velocity (deg/s) — can be used for logging. */
  public double getProfiledVelocity() {
    return currentState.velocity;
  }

  /** Hard-reset the internal state (e.g. on mode switch). */
  public void reset(double positionDegs) {
    currentState = new TrapezoidProfile.State(positionDegs, 0.0);
  }

  /**
   * Hard-reset the internal state, preserving an existing velocity (e.g. when the turret is already
   * moving).
   */
  public void reset(double positionDegs, double velocityDegsPerSec) {
    currentState = new TrapezoidProfile.State(positionDegs, velocityDegsPerSec);
  }
}
