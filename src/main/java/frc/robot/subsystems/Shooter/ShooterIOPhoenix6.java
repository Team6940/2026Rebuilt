package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOPhoenix6 implements ShooterIO {

  private final TalonFX flywheel = new TalonFX(MotorIDs.ShooterMotorID, new CANBus("canivore"));
  private final TalonFX follower =
      new TalonFX(MotorIDs.ShooterFollowerMotorID, new CANBus("canivore"));
  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final Follower followerRequest =
      new Follower(MotorIDs.ShooterMotorID, ShooterConstants.FollowerAlignment);

  public ShooterIOPhoenix6() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = ShooterConstants.ShooterRatio;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 800.;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -800.;
    config.Slot0.kP = ShooterConstants.kP;
    config.Slot0.kI = ShooterConstants.kI;
    config.Slot0.kD = ShooterConstants.kD;
    config.Slot0.kV = ShooterConstants.kV;
    config.Slot0.kS = ShooterConstants.kS;
    config.Slot0.kA = ShooterConstants.kA;

    config.MotionMagic.MotionMagicAcceleration = ShooterConstants.MotionMagicAcceleration;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.ShooterSupplyCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ShooterConstants.ShooterStatorCurrentLimit;

    config.MotorOutput.Inverted = ShooterConstants.Inverted;

    flywheel.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    follower.setControl(followerRequest);
  }

  @Override
  public void setVoltage(double volts) {
    flywheel.setVoltage(volts);
  }

  /**
   * Set the shooter velocity in RPS using torque-aware closed loop. Feedforward is scaled with
   * target RPS so you don’t apply torque FF when idle.
   */
  @Override
  public void setRPS(double rps) {
    if (rps == 0.0) {
      flywheel.stopMotor();
      return;
    }

    // Torque feedforward scaled by target RPS
    double ffAmps = ShooterConstants.kTorqueFFPerRPS * rps;

    // Alternatively, consider using a static torque FF:
    // double ffAmps = ShooterConstants.kT;

    flywheel.setControl(velocityRequest.withVelocity(rps).withFeedForward(ffAmps));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                flywheel.getMotorVoltage(), flywheel.getSupplyCurrent(), flywheel.getVelocity())
            .isOK();

    inputs.motorVoltageVolts = flywheel.getMotorVoltage().getValueAsDouble();
    inputs.motorSupplyCurrentAmps = flywheel.getSupplyCurrent().getValueAsDouble();
    inputs.motorStatorCurrentAmps = flywheel.getStatorCurrent().getValueAsDouble();
    inputs.motorTorqueCurrentAmps = flywheel.getTorqueCurrent().getValueAsDouble();

    inputs.shooterVelocityRPS = flywheel.getVelocity().getValueAsDouble();

    inputs.followerConnected =
        BaseStatusSignal.refreshAll(
                follower.getMotorVoltage(), follower.getSupplyCurrent(), follower.getVelocity())
            .isOK();

    inputs.followerVoltageVolts = follower.getMotorVoltage().getValueAsDouble();
    inputs.followerSupplyCurrentAmps = follower.getSupplyCurrent().getValueAsDouble();
    inputs.followerStatorCurrentAmps = follower.getStatorCurrent().getValueAsDouble();
    inputs.followerTorqueCurrentAmps = follower.getTorqueCurrent().getValueAsDouble();
    inputs.followerVelocityRPS = follower.getVelocity().getValueAsDouble();
  }
}
