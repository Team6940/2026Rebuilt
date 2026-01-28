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

  private final TalonFX flywheel = new TalonFX(MotorIDs.ShooterMotorID, CANBus.roboRIO());
  private final TalonFX follower = new TalonFX(MotorIDs.ShooterFollowerMotorID, CANBus.roboRIO());
  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final Follower followerRequest =
      new Follower(MotorIDs.ShooterMotorID, ShooterConstants.FollowerAlignment);

  public ShooterIOPhoenix6() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = ShooterConstants.ShooterRatio;
    config.Slot0.kP = ShooterConstants.kP;
    config.Slot0.kI = ShooterConstants.kI;
    config.Slot0.kD = ShooterConstants.kD;

    // Disable kV/kS: torque control focuses on current domain.
    config.Slot0.kV = 0.0;
    config.Slot0.kS = 0.0;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.ShooterSupplyCurrentLimit;

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
    inputs.motorCurrentAmps = flywheel.getSupplyCurrent().getValueAsDouble();
    inputs.shooterVelocityRPS = flywheel.getVelocity().getValueAsDouble();
  }
}
