package frc.robot.subsystems.Hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
  public static HoodSubsystem m_instance;

  public static HoodSubsystem getInstance() {
    return m_instance == null ? m_instance = new HoodSubsystem() : m_instance;
  }

  public enum HoodMode {
    HYBRID,
    MANUAL
  }

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private HoodMode mode = HoodMode.HYBRID;
  private double autoSetpointDegs = HoodConstants.IdlePosition;
  private double manualSetpointDegs = HoodConstants.IdlePosition;
  private double targetPositionDegs = HoodConstants.IdlePosition;
  private double operatorInputScalar = 0.0;

  public HoodSubsystem() {
    if (Robot.isReal()) {
      io = new HoodIOPhoenix6();
    } else {
      io = new HoodIO() {};
    }
  }

  public void resetPosition() {
    io.resetPosition(HoodConstants.IdlePosition);
  }

  public void setPosition(double positionDegrees) {
    positionDegrees = clamp(positionDegrees);
    io.setPosition(positionDegrees);
  }

  public void setModeHybrid() {
    mode = HoodMode.HYBRID;
    autoSetpointDegs = clamp(autoSetpointDegs);
  }

  public void setModeManual() {
    mode = HoodMode.MANUAL;
    manualSetpointDegs = clamp(manualSetpointDegs);
  }

  public HoodMode getMode() {
    return mode;
  }

  public void setAutoSetpoint(double positionDegrees) {
    autoSetpointDegs = clamp(positionDegrees);
  }

  public void setManualSetpoint(double positionDegrees) {
    manualSetpointDegs = clamp(positionDegrees);
  }

  public void setOperatorInputScalar(double scalar) {
    operatorInputScalar = MathUtil.clamp(scalar, -1.0, 1.0);
  }

  public void nudgeManualSetpoint(double scalarDelta) {
    manualSetpointDegs =
        clamp(manualSetpointDegs + scalarDelta * HoodConstants.HoodManualSensitivity);
  }

  public double getTargetPositionDegs() {
    return targetPositionDegs;
  }

  public boolean isAtTargetPosition() {
    return MathUtil.isNear(
        targetPositionDegs, inputs.hoodPositionDegrees, HoodConstants.HoodPositionToleranceDegs);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    switch (mode) {
      case HYBRID -> handleHybrid();
      case MANUAL -> handleManual();
    }

    Logger.processInputs("Hood", inputs);
    Logger.recordOutput("Hood/Mode", mode.toString());
    Logger.recordOutput("Hood/TargetPositionDegs", targetPositionDegs);
    Logger.recordOutput("Hood/IsAtTarget", isAtTargetPosition());
    Logger.recordOutput("Hood/AutoSetpointDegs", autoSetpointDegs);
    Logger.recordOutput("Hood/ManualSetpointDegs", manualSetpointDegs);
    Logger.recordOutput("Hood/OperatorInputScalar", operatorInputScalar);
  }

  private void handleHybrid() {
    targetPositionDegs =
        clamp(autoSetpointDegs + operatorInputScalar * HoodConstants.HoodHybridRangeDegs);
    io.setPosition(targetPositionDegs);
  }

  private void handleManual() {
    nudgeManualSetpoint(operatorInputScalar);
    targetPositionDegs = clamp(manualSetpointDegs);
    io.setPosition(targetPositionDegs);
  }

  private double clamp(double positionDegrees) {
    return MathUtil.clamp(positionDegrees, HoodConstants.MinDegs, HoodConstants.MaxDegs);
  }
}
