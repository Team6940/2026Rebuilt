package frc.robot.subsystems.Feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.MotorIDs;

public class FeederIOPhoenix6 implements FeederIO {
  private final TalonFX turntableMotor =
      new TalonFX(MotorIDs.FeederTurntableMotorID, CANBus.roboRIO());
  private final TalonFX feedMotor = new TalonFX(MotorIDs.FeederFeedMotorID, new CANBus("canivore"));

  private final VelocityVoltage turntableVelocityRequest =
      new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC feedVelocityRequest = new VelocityTorqueCurrentFOC(0);

  public FeederIOPhoenix6() {
    turntableMotorConfig();
    feedMotorConfig();
  }

  private void turntableMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = FeederConstants.TurntableRatio;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.Slot0.kP = FeederConstants.TurntablekP;
    config.Slot0.kI = FeederConstants.TurntablekI;
    config.Slot0.kD = FeederConstants.TurntablekD;
    config.Slot0.kV = FeederConstants.TurntablekV;
    config.Slot0.kS = FeederConstants.TurntablekS;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = FeederConstants.TurntableSupplyCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit=FeederConstants.TurntableStatorCurrentLimit;
    config.MotorOutput.Inverted = FeederConstants.TurntableInverted;
    turntableMotor.getConfigurator().apply(config);
  }

  private void feedMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = FeederConstants.FeedRatio;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 800.;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -200.;
    config.Slot0.kP = FeederConstants.FeedkP;
    config.Slot0.kI = FeederConstants.FeedkI;
    config.Slot0.kD = FeederConstants.FeedkD;
    config.Slot0.kV = FeederConstants.FeedkV;
    config.Slot0.kS = FeederConstants.FeedkS;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = FeederConstants.FeedSupplyCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = FeederConstants.FeedStatorCurrentLimit;

    config.MotorOutput.Inverted = FeederConstants.FeedInverted;
    feedMotor.getConfigurator().apply(config);
  }

  @Override
  public void setTurntableVoltage(double voltage) {
    turntableMotor.setVoltage(voltage);
  }

  @Override
  public void setTurntableRPS(double rps) {
    if (rps == 0) {
      turntableMotor.stopMotor();
    } else {
      turntableMotor.setControl(turntableVelocityRequest.withVelocity(rps));
    }
  }

  @Override
  public void setFeedVoltage(double voltage) {
    feedMotor.setVoltage(voltage);
  }

  @Override
  public void setFeedRPS(double rps) {
    if (rps == 0) {
      feedMotor.stopMotor();
    } else {
      feedMotor.setControl(feedVelocityRequest.withVelocity(rps));
    }
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.turntableMotorConnected =
        BaseStatusSignal.refreshAll(
                turntableMotor.getMotorVoltage(),
                turntableMotor.getSupplyCurrent(),
                turntableMotor.getVelocity())
            .isOK();

    inputs.turntableMotorVoltageVolts = turntableMotor.getMotorVoltage().getValueAsDouble();
    inputs.turntableMotorCurrentAmps = turntableMotor.getStatorCurrent().getValueAsDouble();
    inputs.turntableVelocityRPS = turntableMotor.getVelocity().getValueAsDouble();

    inputs.feedMotorConnected =
        BaseStatusSignal.refreshAll(
                feedMotor.getMotorVoltage(), feedMotor.getStatorCurrent(), feedMotor.getVelocity())
            .isOK();

    inputs.feedMotorVoltageVolts = feedMotor.getMotorVoltage().getValueAsDouble();
    inputs.feedMotorSupplyCurrentAmps = feedMotor.getSupplyCurrent().getValueAsDouble();
    inputs.feedMotorStatorCurrentAmps = feedMotor.getStatorCurrent().getValueAsDouble();
    inputs.feedVelocityRPS = feedMotor.getVelocity().getValueAsDouble();
  }
}
