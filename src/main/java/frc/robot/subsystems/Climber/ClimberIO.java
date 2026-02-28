package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  public default void setVoltage(double voltage) {}

  public default void setRotation(double positionRotations) {}

  public default void resetPosition(double positionRotations) {}

  public default void zeroClimberPosition() {
    resetPosition(0.0);
  }

  @AutoLog
  public class ClimberIOInputs {
    public boolean motorConnected = false;

    public double motorVoltageVolts;
    public double motorCurrentAmps;

    public double climberPositionRotations;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}
}
