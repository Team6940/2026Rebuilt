package frc.robot.subsystems.Stretcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.StretcherConstants;

public class StretcherIOPhoenix6 implements StretcherIO {
  private static TalonFX motor = new TalonFX(MotorIDs.StretcherMotorID, CANBus.roboRIO());

  private static MotionMagicVoltage m_request = new MotionMagicVoltage(0.).withEnableFOC(true);

  public StretcherIOPhoenix6() {
    motorConfig();
  }

  private void motorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = StretcherConstants.StretcherRatio;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.Slot0.kP = StretcherConstants.kP;
    config.Slot0.kI = StretcherConstants.kI;
    config.Slot0.kD = StretcherConstants.kD;
    config.Slot0.kV = StretcherConstants.kV;
    config.Slot0.kS = StretcherConstants.kS;
    // config.Slot0.kG = StretcherConstants.kG;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = StretcherConstants.StretcherSupplyCurrentLimit;

    config.MotorOutput.Inverted = StretcherConstants.Inverted;

    // config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.MotionMagic.MotionMagicCruiseVelocity = StretcherConstants.MaxVelocity;
    config.MotionMagic.MotionMagicAcceleration = StretcherConstants.Acceleration;

    // config.MotorOutput.DutyCycleNeutralDeadband = StretcherConstants.Deadband;
    motor.getConfigurator().apply(config);

    motor.setPosition(StretcherConstants.IdlePosition); // initialize to idle (0 rotations)
    // zeroStretcherPosition();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setPosition(double positionRotations) {
    motor.setControl(m_request.withPosition(positionRotations));
  }

  @Override
  public void resetPosition(double positionRotations) {
    motor.setPosition(positionRotations);
  }

  @Override
  public void updateInputs(StretcherIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                motor.getMotorVoltage(), motor.getSupplyCurrent(), motor.getVelocity())
            .isOK();

    inputs.motorVoltageVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.stretcherPositionRotations = motor.getPosition().getValueAsDouble();
  }
}
