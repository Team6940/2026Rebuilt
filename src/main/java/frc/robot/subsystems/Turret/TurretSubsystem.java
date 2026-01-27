package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  public static TurretSubsystem m_instance;

  public static TurretSubsystem getInstance() {
    return m_instance == null ? m_instance = new TurretSubsystem() : m_instance;
  }

  public enum TurretMode {
    AUTO,
    MANUAL
  }

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private TurretMode mode = TurretMode.AUTO;
  private double autoSetpointDegs = TurretConstants.IdlePosition;
  private double manualSetpointDegs = TurretConstants.IdlePosition;
  private double targetPositionDegs = TurretConstants.IdlePosition;

  private Rotation2d fieldRelativeRotation2d = new Rotation2d();
  private Pose2d lastRobotPose = new Pose2d();

  public TurretSubsystem() {
    if (Robot.isReal()) {
      io = new TurretIOPhoenix6();
    } else {
      // TODO: Implement simulation code here
      io = new TurretIO() {};
    }
  }

  public void resetPosition() {
    io.resetPosition(TurretConstants.IdlePosition);
  }

  public void setPosition(double positionDegrees) {
    positionDegrees = clamp(positionDegrees);
    io.setPosition(positionDegrees);
  }

  public void setModeAuto() {
    mode = TurretMode.AUTO;
    autoSetpointDegs = clamp(autoSetpointDegs);
  }

  public void setModeManual() {
    mode = TurretMode.MANUAL;
    manualSetpointDegs = clamp(manualSetpointDegs);
  }

  public TurretMode getMode() {
    return mode;
  }

  public void setAutoSetpoint(double positionDegrees) {
    autoSetpointDegs = clamp(positionDegrees);
  }

  /*
   * Sets the turret to a field-relative angle by calculating the
   * turret-relative angle based on the robot's current pose.
   */
  public void setAutofieldRelativeRotation2d(Rotation2d fieldAngle, Pose2d robotPose) {
    fieldRelativeRotation2d = fieldAngle;
    lastRobotPose = robotPose;
    Rotation2d turretRelative = fieldAngle.minus(robotPose.getRotation());
    Logger.recordOutput("Turret/FieldRelativeRotation2d", fieldRelativeRotation2d.getDegrees());
    setAutoSetpoint(turretRelative.getDegrees());
  }

  public void setManualSetpoint(double positionDegrees) {
    manualSetpointDegs = clamp(positionDegrees);
  }

  public void nudgeManualSetpoint(double scalarDelta) {
    manualSetpointDegs =
        clamp(manualSetpointDegs + scalarDelta * TurretConstants.TurretManualSensitivity);
  }

  public double getTargetPositionDegs() {
    return targetPositionDegs;
  }

  public Rotation2d getTurretFieldAngle(Pose2d robotPose) {
    return robotPose.getRotation().plus(Rotation2d.fromDegrees(inputs.turretPositionDegrees));
  }

  public boolean isAtTargetPosition() {
    return MathUtil.isNear(
        targetPositionDegs,
        inputs.turretPositionDegrees,
        TurretConstants.TurretPositionToleranceDegs);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    switch (mode) {
      case AUTO -> handleAuto();
      case MANUAL -> handleManual();
    }

    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/Mode", mode.toString());
    Logger.recordOutput("Turret/TargetPositionDegs", targetPositionDegs);
    Logger.recordOutput("Turret/IsAtTarget", isAtTargetPosition());
    Logger.recordOutput("Turret/AutoSetpointDegs", autoSetpointDegs);
    Logger.recordOutput("Turret/ManualSetpointDegs", manualSetpointDegs);
    Logger.recordOutput("Turret/FieldTargetDegs", fieldRelativeRotation2d.getDegrees());
    Logger.recordOutput("Turret/RobotHeadingDegs", lastRobotPose.getRotation().getDegrees());
  }

  private void handleAuto() {
    targetPositionDegs = clamp(autoSetpointDegs);
    io.setPosition(targetPositionDegs);
  }

  private void handleManual() {
    targetPositionDegs = clamp(manualSetpointDegs);
    io.setPosition(targetPositionDegs);
  }

  private double clamp(double positionDegrees) {
    return MathUtil.clamp(positionDegrees, TurretConstants.MinDegs, TurretConstants.MaxDegs);
  }
}
