package frc.robot.subsystems.Stretcher;

import org.littletonrobotics.junction.AutoLog;

public interface StretcherIO {
    default public void setVoltage(double voltage) {
    }

    default public void setPosition(double position) {
    }

    default public void resetPosition(double position) {
    }

    default public void zeroStretcherPosition() {
        resetPosition(0.);
    }

    @AutoLog
    public class StretcherIOInputs {
        public boolean motorConnected = false;

        public double motorVoltageVolts;
        public double motorCurrentAmps;

        public double StretcherPositionRadians = 0.;
        public double StretcherRotationDegrees = 0.;
    }

    default public void updateInputs(StretcherIOInputs inputs) {
    }
}
