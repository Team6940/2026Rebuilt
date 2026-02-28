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
  /** Additional exponential smoother applied to the low-pass filtered derivative to make it more stable. */
  private double stableDerivative = 0.0;

  /** Lead multiplier: commanded = setpoint + derivative * leadIndex */
  private double leadIndex;

  /**
   * Low-pass filter coefficient in [0, 1). Higher values smooth the derivative more but add lag.
   * 0.0 = no filtering (raw finite-difference), 0.8 = heavily smoothed.
   */
  private final double filterAlpha;

  /** Exponential smoothing alpha for the "stabler" stage. Higher = more smoothing (0..1). */
  private double stablerAlpha = 0.9;

  /** If the absolute (stable) derivative is below this, no lead correction will be applied. */
  private double minDerivativeForLead = 15.;

  /**
   * @param leadIndex Multiplier applied to the setpoint derivative (seconds). Tune this so that
   *     the lead correction roughly equals the hardware's closed-loop lag.
   * @param filterAlpha Low-pass smoothing on the derivative [0, 1). Recommended: 0.6–0.8.
   */
  public SetpointLeadCompensator(double leadIndex, double filterAlpha) {
    this.leadIndex = leadIndex;
    this.filterAlpha = filterAlpha;
  }

  /**
   * Full constructor allowing tuning of the extra stabilizer and minimum derivative threshold.
   *
   * @param leadIndex multiplier applied to the setpoint derivative (seconds)
   * @param filterAlpha low-pass smoothing on the derivative [0,1)
   * @param stablerAlpha extra exponential smoothing alpha applied to the filtered derivative [0,1)
   * @param minDerivativeForLead minimum absolute derivative required to apply lead (units/sec)
   */
  public SetpointLeadCompensator(
      double leadIndex, double filterAlpha, double stablerAlpha, double minDerivativeForLead) {
    this.leadIndex = leadIndex;
    this.filterAlpha = filterAlpha;
    this.stablerAlpha = stablerAlpha;
    this.minDerivativeForLead = minDerivativeForLead;
  }

  /** Convenience constructor with a stronger default filter alpha of 0.85. */
  public SetpointLeadCompensator(double leadIndex) {
    this(leadIndex, 0.85);
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
      // Use the more stable derivative if available
      return rawSetpoint + stableDerivative * leadIndex;
    }

    // Finite-difference derivative
    double rawDerivative = (rawSetpoint - lastSetpoint) / dt;

    // Low-pass filter to reduce noise amplification
    filteredDerivative = filterAlpha * filteredDerivative + (1.0 - filterAlpha) * rawDerivative;

    // Extra smoothing stage (stabler) to make the derivative less jumpy for noisy inputs.
    stableDerivative = stablerAlpha * stableDerivative + (1.0 - stablerAlpha) * filteredDerivative;

    lastSetpoint = rawSetpoint;
    lastTimestamp = now;

    // If the derivative is too small, don't apply lead — just command the raw setpoint.
    if (Math.abs(stableDerivative) < minDerivativeForLead) {
      return rawSetpoint;
    }

    return rawSetpoint + stableDerivative * leadIndex;
  }

  /** Resets internal state (call on command initialize). */
  public void reset() {
    lastSetpoint = Double.NaN;
    lastTimestamp = Double.NaN;
    filteredDerivative = 0.0;
    stableDerivative = 0.0;
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
    // Return the stabilized derivative for logging/telemetry (smoother than the raw filtered value).
    return stableDerivative;
  }

  /** Set the minimum absolute derivative required to apply lead. Use 0 to always apply lead. */
  public void setMinDerivativeForLead(double minDerivativeForLead) {
    this.minDerivativeForLead = minDerivativeForLead;
  }

  public double getMinDerivativeForLead() {
    return minDerivativeForLead;
  }

  /** Tuning for the extra stabilizer stage (alpha in [0,1)). Higher values = more smoothing. */
  public void setStablerAlpha(double stablerAlpha) {
    this.stablerAlpha = stablerAlpha;
  }

  public double getStablerAlpha() {
    return stablerAlpha;
  }
}
