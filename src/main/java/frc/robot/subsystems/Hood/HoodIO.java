package frc.robot.subsystems.Hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  public default void setVoltage(double voltage) {}

  public default void setPosition(double positionDegrees) {}

  public default void resetPosition(double positionDegrees) {}

  public default void zeroHoodPosition() {
    resetPosition(0.0);
  }

  @AutoLog
  public class HoodIOInputs {
    public boolean motorConnected = false;

    public double motorVoltageVolts;
    public double motorCurrentAmps;

    public double hoodPositionDegrees;
  }

  public default void updateInputs(HoodIOInputs inputs) {}
}
