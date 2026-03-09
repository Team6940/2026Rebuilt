package frc.robot.subsystems.Feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public class FeederIOInputs {
    // Turntable motor
    public boolean turntableMotorConnected = false;
    public double turntableMotorVoltageVolts = 0.0;
    public double turntableMotorCurrentAmps = 0.0;
    public double turntableVelocityRPS = 0.0;

    // Feed motor (upward feeding)
    public boolean feedMotorConnected = false;
    public double feedMotorVoltageVolts = 0.0;
    public double feedMotorSupplyCurrentAmps = 0.0;
    public double feedMotorStatorCurrentAmps = 0.0;
    public double feedVelocityRPS = 0.0;
  }

  /** Set turntable motor voltage */
  public default void setTurntableVoltage(double voltage) {}

  /** Set turntable motor velocity in RPS */
  public default void setTurntableRPS(double rps) {}

  /** Set feed motor voltage */
  public default void setFeedVoltage(double voltage) {}

  /** Set feed motor velocity in RPS */
  public default void setFeedRPS(double rps) {}

  /** Update inputs */
  public default void updateInputs(FeederIOInputs inputs) {}
}
