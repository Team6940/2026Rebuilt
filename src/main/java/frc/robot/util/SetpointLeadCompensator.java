package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * Computes a lead-compensated setpoint by estimating the derivative (rate of change) of the
 * setpoint over time and adding (derivative * leadIndex) to the raw setpoint.
 *
 * <p>This prevents mechanical "layback" — the condition where hardware lags behind a rapidly
 * moving setpoint because it is always chasing a target that has already moved. By commanding an
 * over-shot target in the direction of motion, the hardware converges on the true setpoint faster.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * SetpointLeadCompensator comp = new SetpointLeadCompensator(0.1);
 * double commanded = comp.calculate(rawSetpointDeg);
 * }</pre>
 */
public class SetpointLeadCompensator {

  private double lastSetpoint = Double.NaN;
  private double lastTimestamp = Double.NaN;
  private double filteredDerivative = 0.0;

  /** Lead multiplier: commanded = setpoint + derivative * leadIndex */
  private double leadIndex;

  /**
   * Low-pass filter coefficient in [0, 1). Higher values smooth the derivative more but add lag.
   * 0.0 = no filtering (raw finite-difference), 0.8 = heavily smoothed.
   */
  private final double filterAlpha;

  /**
   * @param leadIndex Multiplier applied to the setpoint derivative (seconds). Tune this so that
   *     the lead correction roughly equals the hardware's closed-loop lag.
   * @param filterAlpha Low-pass smoothing on the derivative [0, 1). Recommended: 0.6–0.8.
   */
  public SetpointLeadCompensator(double leadIndex, double filterAlpha) {
    this.leadIndex = leadIndex;
    this.filterAlpha = filterAlpha;
  }

  /** Convenience constructor with default filter alpha of 0.7. */
  public SetpointLeadCompensator(double leadIndex) {
    this(leadIndex, 0.7);
  }

  /**
   * Feed in the latest raw setpoint and receive the lead-compensated setpoint.
   *
   * @param rawSetpoint The ideal setpoint at this moment (degrees, rotations, etc.)
   * @return The lead-compensated setpoint to actually command to hardware.
   */
  public double calculate(double rawSetpoint) {
    double now = Timer.getFPGATimestamp();

    if (Double.isNaN(lastSetpoint) || Double.isNaN(lastTimestamp)) {
      // First call — no derivative available yet
      lastSetpoint = rawSetpoint;
      lastTimestamp = now;
      return rawSetpoint;
    }

    double dt = now - lastTimestamp;

    // If dt is too large (e.g. robot was disabled, loop stall), the finite-difference
    // would produce a huge spike. Treat it as a fresh start instead.
    if (dt > 0.5) {
      lastSetpoint = rawSetpoint;
      lastTimestamp = now;
      filteredDerivative = 0.0;
      return rawSetpoint;
    }

    if (dt < 1e-6) {
      // Identical timestamp — reuse last filtered derivative, don't update state
      return rawSetpoint + filteredDerivative * leadIndex;
    }

    // Finite-difference derivative
    double rawDerivative = (rawSetpoint - lastSetpoint) / dt;

    // Low-pass filter to reduce noise amplification
    filteredDerivative = filterAlpha * filteredDerivative + (1.0 - filterAlpha) * rawDerivative;

    lastSetpoint = rawSetpoint;
    lastTimestamp = now;

    return rawSetpoint + filteredDerivative * leadIndex;
  }

  /** Resets internal state (call on command initialize). */
  public void reset() {
    lastSetpoint = Double.NaN;
    lastTimestamp = Double.NaN;
    filteredDerivative = 0.0;
  }

  /** Update the lead index at runtime for tuning. */
  public void setLeadIndex(double leadIndex) {
    this.leadIndex = leadIndex;
  }

  public double getLeadIndex() {
    return leadIndex;
  }

  /** Returns the last computed (filtered) derivative for logging. */
  public double getFilteredDerivative() {
    return filteredDerivative;
  }
}
