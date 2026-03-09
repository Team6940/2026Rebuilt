package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  public default void setVoltage(double voltage) {}

  public default void setPosition(double positionDegrees) {}

  /**
   * Commands the turret to a position with an explicit velocity feedforward.
   *
   * @param positionDegrees target turret angle in degrees
   * @param velocityDegsPerSec feedforward velocity in degrees/second (trapezoidal profile + chassis
   *     omega compensation)
   */
  public default void setPositionWithVelocity(double positionDegrees, double velocityDegsPerSec) {}

  public default void resetPosition(double positionDegrees) {}

  public default void zeroTurretPosition() {
    resetPosition(0.0);
  }

  @AutoLog
  public class TurretIOInputs {
    public boolean motorConnected = false;
    public boolean encoderConnected = false;
    public boolean encoder2Connected = false;

    public double motorVoltageVolts;
    public double motorCurrentAmps;

    public double encoderPositionDegrees;
    public double encoder2PositionDegrees;
    public double turretPositionDegrees;

    public enum EncoderMagnetHealth {
      GOOD,
      RISKY,
      BAD,
      INVALID
    }

    public EncoderMagnetHealth encoderMagnetHealth = EncoderMagnetHealth.BAD;
    public EncoderMagnetHealth encoder2MagnetHealth = EncoderMagnetHealth.BAD;
  }

  public default void updateInputs(TurretIOInputs inputs) {}
}
