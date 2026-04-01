package frc.robot.subsystems.Feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class FeederSubsystem extends SubsystemBase {
  public static FeederSubsystem m_instance;

  public static FeederSubsystem getInstance() {
    return m_instance == null ? m_instance = new FeederSubsystem() : m_instance;
  }

  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private enum TurntableControlMode {
    VELOCITY,
    VOLTAGE
  }

  private enum TurntableJamState {
    FORWARD,
    REVERSE
  }

  private final Debouncer turntableJamDebouncer =
      new Debouncer(FeederConstants.TurntableJamDetectTimeSecs);

  private TurntableControlMode turntableControlMode = TurntableControlMode.VELOCITY;
  private TurntableJamState turntableJamState = TurntableJamState.FORWARD;
  private double targetTurntableRPS = 0.0;
  private double appliedTurntableRPS = 0.0;
  private double targetTurntableVoltage = 0.0;
  private double turntableJamReverseEndTimeSecs = 0.0;
  private double targetFeedRPS = 0.0;

  public FeederSubsystem() {
    if (Robot.isReal()) {
      io = new FeederIOPhoenix6();
    } else {
      io = new FeederIO() {};
    }
  }

  /**
   * Set turntable motor velocity in RPS
   * @param rps Target velocity in rotations per second
   */
  public void setTurntableRPS(double rps) {
    turntableControlMode = TurntableControlMode.VELOCITY;
    targetTurntableRPS = rps;
    targetTurntableVoltage = 0.0;
    if (rps <= 0.0) {
      clearTurntableJamState();
    }
    updateTurntableCommand();
  }

  /**
   * Set feed motor velocity in RPS
   * @param rps Target velocity in rotations per second
   */
  public void setFeedRPS(double rps) {
    targetFeedRPS = rps;
    io.setFeedRPS(rps);
  }

  /**
   * Set both motors to the same velocity
   * @param turntableRPS Turntable motor velocity in RPS
   * @param feedRPS Feed motor velocity in RPS
   */
  public void setRPS(double turntableRPS, double feedRPS) {
    setTurntableRPS(turntableRPS);
    setFeedRPS(feedRPS);
  }

  /**
   * Check if turntable motor is at target velocity
   * @return True if turntable motor is within tolerance of target
   */
  public boolean isTurntableAtTargetRPS() {
    return MathUtil.isNear(
        targetTurntableRPS,
        inputs.turntableVelocityRPS,
        FeederConstants.TurntableVelocityToleranceRPS);
  }

  /**
   * Check if feed motor is at target velocity
   * @return True if feed motor is within tolerance of target
   */
  public boolean isFeedAtTargetRPS() {
    return MathUtil.isNear(
        targetFeedRPS, inputs.feedVelocityRPS, FeederConstants.FeedVelocityToleranceRPS);
  }

  /**
   * Check if both motors are at target velocity
   * @return True if both motors are within tolerance of target
   */
  public boolean isAtTargetRPS() {
    return isTurntableAtTargetRPS() && isFeedAtTargetRPS();
  }

  /**
   * Get current turntable motor velocity
   * @return Current turntable velocity in RPS
   */
  public double getTurntableRPS() {
    return inputs.turntableVelocityRPS;
  }

  /**
   * Get current feed motor velocity
   * @return Current feed velocity in RPS
   */
  public double getFeedRPS() {
    return inputs.feedVelocityRPS;
  }

  /**
   * Set turntable motor voltage
   * @param voltage Voltage to apply
   */
  public void setTurntableVoltage(double voltage) {
    turntableControlMode = TurntableControlMode.VOLTAGE;
    targetTurntableRPS = 0.0;
    targetTurntableVoltage = voltage;
    appliedTurntableRPS = 0.0;
    clearTurntableJamState();
    io.setTurntableVoltage(voltage);
  }

  /**
   * Set feed motor voltage
   * @param voltage Voltage to apply
   */
  public void setFeedVoltage(double voltage) {
    targetFeedRPS = 0.0;
    io.setFeedVoltage(voltage);
  }

  /**
   * Stop both motors
   */
  public void stop() {
    setRPS(0.0, 0.0);
  }

  @Override
  public void periodic() {
    processLog();
    processDashboard();
  }

  private void processLog() {
    io.updateInputs(inputs);
    processTurntableJamState();
    updateTurntableCommand();
    Logger.processInputs("Feeder", inputs);
    Logger.recordOutput("Feeder/TargetTurntableRPS", targetTurntableRPS);
    Logger.recordOutput("Feeder/AppliedTurntableRPS", appliedTurntableRPS);
    Logger.recordOutput("Feeder/TargetTurntableVoltage", targetTurntableVoltage);
    Logger.recordOutput("Feeder/TargetFeedRPS", targetFeedRPS);
    Logger.recordOutput("Feeder/IsTurntableAtTargetRPS", isTurntableAtTargetRPS());
    Logger.recordOutput("Feeder/IsFeedAtTargetRPS", isFeedAtTargetRPS());
    Logger.recordOutput("Feeder/IsAtTargetRPS", isAtTargetRPS());
    Logger.recordOutput("Feeder/TurntableControlMode", turntableControlMode.toString());
    Logger.recordOutput("Feeder/TurntableJamState", turntableJamState.toString());
    Logger.recordOutput(
        "Feeder/TurntableJamCurrentHigh",
        inputs.turntableMotorCurrentAmps >= FeederConstants.TurntableJamCurrentThresholdAmps);
  }

  private void processDashboard() {
  }

  private void processTurntableJamState() {
    if (turntableControlMode != TurntableControlMode.VELOCITY || targetTurntableRPS <= 0.0) {
      turntableJamDebouncer.calculate(false);
      if (targetTurntableRPS <= 0.0) {
        clearTurntableJamState();
      }
      return;
    }

    double nowSecs = Timer.getFPGATimestamp();
    if (turntableJamState == TurntableJamState.REVERSE) {
      turntableJamDebouncer.calculate(false);
      if (nowSecs >= turntableJamReverseEndTimeSecs) {
        turntableJamState = TurntableJamState.FORWARD;
      }
      return;
    }

    boolean jamDetected =
        turntableJamDebouncer.calculate(
            inputs.turntableMotorCurrentAmps >= FeederConstants.TurntableJamCurrentThresholdAmps);
    if (jamDetected) {
      turntableJamState = TurntableJamState.REVERSE;
      turntableJamReverseEndTimeSecs = nowSecs + FeederConstants.TurntableJamReverseTimeSecs;
    }
  }

  private void updateTurntableCommand() {
    if (turntableControlMode != TurntableControlMode.VELOCITY) {
      return;
    }

    appliedTurntableRPS =
        turntableJamState == TurntableJamState.REVERSE
            ? FeederConstants.TurntableJamReverseRPS
            : targetTurntableRPS;
    io.setTurntableRPS(appliedTurntableRPS);
  }

  private void clearTurntableJamState() {
    turntableJamState = TurntableJamState.FORWARD;
    turntableJamReverseEndTimeSecs = 0.0;
    turntableJamDebouncer.calculate(false);
  }
}

