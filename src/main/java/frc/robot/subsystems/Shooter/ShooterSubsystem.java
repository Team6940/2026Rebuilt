package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  public static ShooterSubsystem m_Instance;

  public static ShooterSubsystem getInstance() {
    return m_Instance == null ? m_Instance = new ShooterSubsystem() : m_Instance;
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double targetRPS = 0;

  public ShooterSubsystem() {
    if (Robot.isReal()) {
      io = new ShooterIOPhoenix6();
    } else {
      // TODO: Implement simulation code here
      io = new ShooterIOPhoenix6();
    }
  }

  public void setRPS(double rps) {
    targetRPS = rps;
    io.setRPS(rps);
  }

  public boolean isAtTargetRps() {
    return MathUtil.isNear(
        targetRPS, inputs.shooterVelocityRPS, ShooterConstants.ShooterVelocityToleranceRPS);
  }

  public double getShooterRPS() {
    return inputs.shooterVelocityRPS;
  }

  public void stop() {
    targetRPS = 0;
    io.setRPS(0);
  }

  public void setVoltage(double voltage) {
    targetRPS = 0;
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    processLog();
    processDashboard();
  }

  private void processLog() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/TargetRPS", targetRPS);
    Logger.recordOutput("Shooter/IsAtTargetRPS", isAtTargetRps());
  }

  private void processDashboard() {
    // TODO: Implement dashboard code here
  }
}
