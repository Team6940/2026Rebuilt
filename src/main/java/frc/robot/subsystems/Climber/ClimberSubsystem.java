package frc.robot.subsystems.Climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  public static ClimberSubsystem m_instance;

  public static ClimberSubsystem getInstance() {
    return m_instance == null ? m_instance = new ClimberSubsystem() : m_instance;
  }

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private double targetPositionDegs = ClimberConstants.IdlePosition;

  public ClimberSubsystem() {
    if (Robot.isReal()) {
      io = new ClimberIOPhoenix6();
    } else {
      // TODO: Implement simulation code here
      io = new ClimberIO() {};
    }
  }

  public void resetPosition() {
    io.resetPosition(ClimberConstants.IdlePosition);
    targetPositionDegs = ClimberConstants.IdlePosition;
  }

  public void setPosition(double positionDegrees) {
    targetPositionDegs = clamp(positionDegrees);
    io.setPosition(targetPositionDegs);
  }

  public void setRetracted() {
    setPosition(ClimberConstants.RetractedPosition);
  }

  public void setExtended() {
    setPosition(ClimberConstants.ExtendedPosition);
  }

  public double getTargetPositionDegs() {
    return targetPositionDegs;
  }

  public double getCurrentPositionDegs() {
    return inputs.climberPositionDegrees;
  }

  public boolean isAtTargetPosition() {
    return MathUtil.isNear(
        targetPositionDegs,
        inputs.climberPositionDegrees,
        ClimberConstants.ClimberPositionToleranceDegs);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void stop() {
    io.setVoltage(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Climber", inputs);
    Logger.recordOutput("Climber/TargetPositionDegs", targetPositionDegs);
    Logger.recordOutput("Climber/IsAtTarget", isAtTargetPosition());
  }

  private double clamp(double positionDegrees) {
    return MathUtil.clamp(positionDegrees, ClimberConstants.MinDegs, ClimberConstants.MaxDegs);
  }
}
