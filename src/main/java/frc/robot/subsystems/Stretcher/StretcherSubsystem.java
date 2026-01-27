package frc.robot.subsystems.Stretcher;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Library.team6940.MUtils2025;
import frc.robot.Constants.StretcherConstants;



public class StretcherSubsystem extends SubsystemBase{
    public static StretcherSubsystem m_instance = null;

    public static StretcherSubsystem getInstance() {
        return m_instance == null ? m_instance = new StretcherSubsystem() : m_instance;
    }

    private final StretcherIO io;
    private final StretcherIOInputsAutoLogged inputs = new StretcherIOInputsAutoLogged();
    private double targetPosition = 0.;

    public StretcherSubsystem() {
        if (Robot.isReal()) {
            io = new StretcherIOPhoenix6();
        } else {
            // TODO: Implement simulation code here
            io = new StretcherIO(){};
        }
    }

    public void zeroStretcherPosition() {
        io.zeroStretcherPosition();
    }

    /**
     * 
     * @param position radians
     */
    public void setPosition(double position) {
        targetPosition = MUtils2025.numberLimit(StretcherConstants.MinDegs, StretcherConstants.MaxDegs, position);
        io.setPosition(targetPosition);
    }

    boolean IsAtTargetPosition() {
        return MathUtil.isNear(targetPosition, inputs.stretcherPositionRadians, StretcherConstants.StretcherPositionToleranceDegs);
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void stop() {
        setPosition(0.);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void processLog() {
        io.updateInputs(inputs);
        Logger.processInputs("Stretcher", inputs);
        Logger.recordOutput("Stretcher/TargetPosition", targetPosition);
        // TODO Logger
    }

    @Override
    public void periodic() {
        processLog();
        processDashboard();
    }

    private void processDashboard() {
        // TODO: Implement dashboard code here
    }
}

