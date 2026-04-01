package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  public static ShooterSubsystem m_Instance;

  public static ShooterSubsystem getInstance() {
    return m_Instance == null ? m_Instance = new ShooterSubsystem() : m_Instance;
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static final double MAX_DASH_RPS = 100.0;
  private final GenericEntry dashRps =
      Shuffleboard.getTab("Shooter")
          .add("RPS", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0.0, "max", MAX_DASH_RPS))
          .getEntry();

  private double targetRPS = 0;
  private double dashboardRps = 0.0;

  public ShooterSubsystem() {
    if (Robot.isReal()) {
      io = new ShooterIOPhoenix6();
    } else {
      io = new ShooterIOPhoenix6();
    }
  }

  public void setRPS(double rps) {
    targetRPS = rps;
    io.setRPS(rps);
  }

  /**
   * Commands the shooter to run at the current dashboard-tuned RPS value. Call this instead of
   * setRPS() during Shuffleboard tuning sessions so the slider value drives the motor.
   */
  public void setDashboardRPS() {
    targetRPS = dashboardRps;
    io.setRPS(dashboardRps);
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
    Logger.recordOutput("Shooter/DashboardRPS", dashboardRps);
  }

  private void processDashboard() {
    dashboardRps = MathUtil.clamp(dashRps.getDouble(0.0), 0.0, MAX_DASH_RPS);
  }
}
