package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public class ShooterIOInputs {
    public boolean motorConnected;
    public double motorVoltageVolts;
    public double motorSupplyCurrentAmps;
    public double motorStatorCurrentAmps;
    public double motorTorqueCurrentAmps;
    public double shooterVelocityRPS;

    public boolean followerConnected;
    public double followerVoltageVolts;
    public double followerSupplyCurrentAmps;
    public double followerStatorCurrentAmps;
    public double followerTorqueCurrentAmps;
    public double followerVelocityRPS;
  }

  public default void setVoltage(double voltage) {}

  public default void setRPS(double rps) {}

  public default void updateInputs(ShooterIOInputs inputs) {}
}
