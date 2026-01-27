package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  public default void setVoltage(double voltage) {}

  public default void setPosition(double positionDegrees) {}

  public default void resetPosition(double positionDegrees) {}

  public default void zeroTurretPosition() {
    resetPosition(0.0);
  }

  @AutoLog
  public class TurretIOInputs {
    public boolean motorConnected = false;

    public double motorVoltageVolts;
    public double motorCurrentAmps;

    public double turretPositionDegrees;
  }

  public default void updateInputs(TurretIOInputs inputs) {}
}
