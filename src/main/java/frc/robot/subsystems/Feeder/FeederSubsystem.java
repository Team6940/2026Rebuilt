package frc.robot.subsystems.Feeder;

import edu.wpi.first.math.MathUtil;
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

  private double targetTurntableRPS = 0.0;
  private double targetFeedRPS = 0.0;

  public FeederSubsystem() {
    if (Robot.isReal()) {
      io = new FeederIOPhoenix6();
    } else {
      // TODO: Implement simulation code here
      io = new FeederIO() {};
    }
  }

  /**
   * Set turntable motor velocity in RPS
   * @param rps Target velocity in rotations per second
   */
  public void setTurntableRPS(double rps) {
    targetTurntableRPS = rps;
    io.setTurntableRPS(rps);
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
    targetTurntableRPS = 0.0;
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
    Logger.processInputs("Feeder", inputs);
    Logger.recordOutput("Feeder/TargetTurntableRPS", targetTurntableRPS);
    Logger.recordOutput("Feeder/TargetFeedRPS", targetFeedRPS);
    Logger.recordOutput("Feeder/IsTurntableAtTargetRPS", isTurntableAtTargetRPS());
    Logger.recordOutput("Feeder/IsFeedAtTargetRPS", isFeedAtTargetRPS());
    Logger.recordOutput("Feeder/IsAtTargetRPS", isAtTargetRPS());
  }

  private void processDashboard() {
    // TODO: Implement dashboard code here
  }
}

