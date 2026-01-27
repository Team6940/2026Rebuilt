package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    
    default public void setVoltage(double voltage){}

    default public void setRPS(double rps){}

    @AutoLog
    public class IntakeIOInputs {
        public boolean motorConnected;
        public double motorVoltageVolts;
        public double motorCurrentAmps;
        public double intakeVelocityRPS;

    }
    default public void updateInputs(IntakeIOInputs inputs){}
}
