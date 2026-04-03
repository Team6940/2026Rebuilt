package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    public static IntakeSubsystem m_instance;
    public static IntakeSubsystem getInstance() {
        return m_instance == null? m_instance = new IntakeSubsystem() : m_instance;
    }

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private double targetRPS = 0.;

    public IntakeSubsystem(){
        if(Robot.isReal()){
            io = new IntakeIOPhoenix6();
        }
        else{
            io = new IntakeIO(){};
        }
    }


    public void setRPS(double rps){
        targetRPS = rps;
        io.setRPS(rps);
    }

    public boolean isAtTargetRPS(){
        return MathUtil.isNear(targetRPS, inputs.intakeVelocityRPS, IntakeConstants.IntakeVelocityToleranceRPS);
    }

    public void stop(){
        setRPS(0.);
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        processLog();
        processDashboard();
    }

    public void processLog(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Intake/TargetRPS", targetRPS);
        Logger.recordOutput("Intake/IsAtTargetRPS", isAtTargetRPS());
    }

    private void processDashboard(){
    }

}
