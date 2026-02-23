package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorIDs;

public class IntakeIOPhoenix6 implements IntakeIO {
  private static final TalonFX motor = new TalonFX(MotorIDs.IntakeMotorID, new CANBus("CANivore"));

  private static final VelocityVoltage dutycycle = new VelocityVoltage(0).withEnableFOC(true);

  public IntakeIOPhoenix6() {
    motorConfig();
  }

  private void motorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = IntakeConstants.IntakeRatio;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.Slot0.kP = IntakeConstants.kP;
    config.Slot0.kI = IntakeConstants.kI;
    config.Slot0.kD = IntakeConstants.kD;
    config.Slot0.kV = IntakeConstants.kV;
    config.Slot0.kS = IntakeConstants.kS;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.IntakeSupplyCurrentLimit;

    config.MotorOutput.Inverted = IntakeConstants.IntakeInverted;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setRPS(double rps) {
    if (rps == 0) {
      motor.stopMotor();
    }
    motor.setControl(dutycycle.withVelocity(rps));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                motor.getMotorVoltage(), motor.getSupplyCurrent(), motor.getVelocity())
            .isOK();

    inputs.motorVoltageVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.intakeVelocityRPS = motor.getVelocity().getValueAsDouble();
  }
}
