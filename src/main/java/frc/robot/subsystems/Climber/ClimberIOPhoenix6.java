package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorIDs;

public class ClimberIOPhoenix6 implements ClimberIO {
  private static TalonFX motor = new TalonFX(MotorIDs.ClimberMotorID, CANBus.roboRIO());
  private static final MotionMagicVoltage request = new MotionMagicVoltage(0.0).withEnableFOC(true);

  public ClimberIOPhoenix6() {
    motorConfig();
  }

  private void motorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = ClimberConstants.ClimberRatio;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.Slot0.kP = ClimberConstants.kP;
    config.Slot0.kI = ClimberConstants.kI;
    config.Slot0.kD = ClimberConstants.kD;
    config.Slot0.kV = ClimberConstants.kV;
    config.Slot0.kS = ClimberConstants.kS;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ClimberConstants.ClimberSupplyCurrentLimit;

    config.MotorOutput.Inverted = ClimberConstants.Inverted;

    config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.MaxVelocity;
    config.MotionMagic.MotionMagicAcceleration = ClimberConstants.Acceleration;

    motor.getConfigurator().apply(config);
    motor.setPosition(ClimberConstants.IdlePosition);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setRotation(double positionRotations) {
    motor.setControl(request.withPosition(positionRotations));
  }

  @Override
  public void resetPosition(double positionRotations) {
    motor.setPosition(positionRotations);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                motor.getMotorVoltage(), motor.getSupplyCurrent(), motor.getPosition())
            .isOK();

    inputs.motorVoltageVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.climberPositionRotations = motor.getPosition().getValueAsDouble();
  }
}
