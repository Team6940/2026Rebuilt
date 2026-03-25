package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * Estimates the rate-of-change (derivative) of a continuously-updated angular setpoint (degrees).
 *
 * <p>Designed for turret velocity-mode feedforward: the outer control loop calls {@code
 * calculate(setpoint)} every control cycle and receives a low-pass-filtered velocity estimate
 * (deg/s). The finite-difference uses {@link MathUtil#inputModulus} on the delta so that a
 * setpoint jump (rewrap event across the ±180° boundary) never produces a huge spike.
 *
 * <p>Actual elapsed time is measured via {@link Timer#getFPGATimestamp()} so the derivative is
 * correct even when the loop period varies (e.g. during brownouts or logging overruns). Large dt
 * gaps (> 0.5 s, e.g. after disable) are treated as a fresh start to avoid one-shot spikes.
 *
 * <p>Architecture mirrors {@link SetpointLeadCompensator}: first-order IIR low-pass on the
 * finite-difference, plus an optional second smoothing stage.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * SetpointDerivative deriv = new SetpointDerivative(0.7);
 * // every loop:
 * double velFFDegsPerSec = deriv.calculate(autoSetpointDeg);
 * }</pre>
 */
public class SetpointDerivative {

  private double lastSetpoint = Double.NaN;
  private double lastTimestamp = Double.NaN;

  /** First-stage IIR low-pass filtered derivative. */
  private double filteredDerivative = 0.0;

  /** Second-stage (stabler) filtered derivative — what's actually returned. */
  private double stableDerivative = 0.0;

  /**
   * Low-pass filter coefficient in [0, 1). Higher → more smoothing, more lag. Recommended:
   * 0.6–0.85.
   */
  private final double filterAlpha;

  /**
   * Second-stage smoothing coefficient in [0, 1). Set higher than filterAlpha for extra stability.
   */
  private final double stablerAlpha;

  /**
   * If |stableDerivative| is below this threshold, return 0 to avoid injecting noise when the
   * setpoint is stationary.
   */
  private final double minDerivative;

  /**
   * Full constructor.
   *
   * @param filterAlpha first-stage IIR coefficient [0, 1)
   * @param stablerAlpha second-stage IIR coefficient [0, 1)
   * @param minDerivative absolute derivative below which 0 is returned (dead-band)
   */
  public SetpointDerivative(double filterAlpha, double stablerAlpha, double minDerivative) {
    this.filterAlpha = filterAlpha;
    this.stablerAlpha = stablerAlpha;
    this.minDerivative = minDerivative;
  }

  /**
   * Convenience constructor: no dead-band.
   *
   * @param filterAlpha first-stage IIR coefficient [0, 1). Recommended: 0.7.
   */
  public SetpointDerivative(double filterAlpha) {
    this(filterAlpha, 0.85, 0.0);
  }

  /**
   * Feed in the latest setpoint and receive the estimated rate-of-change.
   *
   * <p>Uses the real FPGA timestamp for dt so accuracy is preserved across variable-length loops.
   * Returns 0 on the first call, and also resets (returns 0) when more than 0.5 s has elapsed
   * since the last call (e.g. robot was disabled).
   *
   * @param setpoint The current setpoint value (degrees).
   * @return Estimated rate-of-change in deg/s.
   */
  public double calculate(double setpoint) {
    double now = Timer.getFPGATimestamp();

    if (Double.isNaN(lastSetpoint) || Double.isNaN(lastTimestamp)) {
      // First call — seed state, no derivative available yet.
      lastSetpoint = setpoint;
      lastTimestamp = now;
      return 0.0;
    }

    double dt = now - lastTimestamp;

    // Large gap (e.g. after disable) — treat as a fresh start to avoid a spike.
    if (dt > 0.5) {
      lastSetpoint = setpoint;
      lastTimestamp = now;
      filteredDerivative = 0.0;
      stableDerivative = 0.0;
      return 0.0;
    }

    if (dt < 1e-6) {
      // Same timestamp — reuse the last result without updating state.
      return Math.abs(stableDerivative) < minDerivative ? 0.0 : stableDerivative;
    }

    double rawDerivative =
        MathUtil.inputModulus(setpoint - lastSetpoint, -180.0, 180.0) / dt;
    lastSetpoint = setpoint;
    lastTimestamp = now;

    // First-stage low-pass
    filteredDerivative = filterAlpha * filteredDerivative + (1.0 - filterAlpha) * rawDerivative;

    // Second-stage (stabler)
    stableDerivative = stablerAlpha * stableDerivative + (1.0 - stablerAlpha) * filteredDerivative;

    return Math.abs(stableDerivative) < minDerivative ? 0.0 : stableDerivative;
  }

  /** Resets internal state (call on mode entry / command initialize). */
  public void reset() {
    lastSetpoint = Double.NaN;
    lastTimestamp = Double.NaN;
    filteredDerivative = 0.0;
    stableDerivative = 0.0;
  }

  /** Returns the most recently computed stable derivative (for logging). */
  public double getStableDerivative() {
    return stableDerivative;
  }
}
