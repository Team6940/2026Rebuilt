package frc.robot.subsystems.Intaker;

import org.littletonrobotics.junction.AutoLog;

public interface IntakerIO {
    
    default public void setVoltage(double voltage){}

    default public void setRPS(double rps){}

    @AutoLog
    public class IntakerIOInputs {
        public boolean motorConnected;
        public double motorVoltageVolts;
        public double motorCurrentAmps;
        public double intakerVelocityRPS;

    }
    default public void updateInputs(IntakerIOInputs inputs){}
}
