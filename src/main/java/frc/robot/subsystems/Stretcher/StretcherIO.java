package frc.robot.subsystems.Stretcher;

import org.littletonrobotics.junction.AutoLog;

public interface StretcherIO {
  public default void setVoltage(double voltage) {}

  public default void setPosition(double position) {}

  public default void resetPosition(double position) {}

  public default void zeroStretcherPosition() {
    resetPosition(0.);
  }

  public default void setCoast() {}

  @AutoLog
  public class StretcherIOInputs {
    public boolean motorConnected = false;

    public double motorVoltageVolts = 0.0;
    public double motorCurrentAmps = 0.0;

    /** Current position in rotations. */
    public double stretcherPositionRotations = 0.0;
  }

  public default void updateInputs(StretcherIOInputs inputs) {}
}
