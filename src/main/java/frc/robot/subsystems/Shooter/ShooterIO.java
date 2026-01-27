package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public class ShooterIOInputs {
    public boolean motorConnected;
    public double motorVoltageVolts;
    public double motorCurrentAmps;
    public double shooterVelocityRPS;
    public boolean sensorFrontGet;
    public boolean sensorRearGet;
  }

  public default void setVoltage(double voltage) {}

  public default void setRPS(double rps) {}

  public default void updateInputs(ShooterIOInputs inputs) {}
}
