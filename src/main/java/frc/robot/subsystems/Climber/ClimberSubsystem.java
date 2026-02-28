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

  private double targetPositionRotations = ClimberConstants.IdlePosition;

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
    targetPositionRotations = ClimberConstants.IdlePosition;
  }

  public void setRotation(double positionRotations) {
    targetPositionRotations = clamp(positionRotations);
    io.setRotation(targetPositionRotations);
  }

  public void setRetracted() {
    setRotation(ClimberConstants.RetractedPosition);
  }

  public void setExtended() {
    setRotation(ClimberConstants.ExtendedPosition);
  }

  public double getTargetPositionRotations() {
    return targetPositionRotations;
  }

  public double getCurrentPositionRotations() {
    return inputs.climberPositionRotations;
  }

  public boolean isAtTargetPosition() {
    return MathUtil.isNear(
        targetPositionRotations,
        inputs.climberPositionRotations,
        ClimberConstants.ClimberPositionToleranceRotations);
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
    Logger.recordOutput("Climber/TargetPositionRotations", targetPositionRotations);
    Logger.recordOutput("Climber/IsAtTarget", isAtTargetPosition());
  }

  private double clamp(double positionRotations) {
    return MathUtil.clamp(positionRotations, ClimberConstants.MinRotations, ClimberConstants.MaxRotations);
  }
}
