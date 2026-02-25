package frc.robot.subsystems.Stretcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
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

    motor.setPosition(
        Units.degreesToRotations(
            StretcherConstants.IdlePosition)); // 1/4rotation, which means 90degs
    // zeroStretcherPosition();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setPosition(double position) {
    // if (position == 0) {
    //     motor.stopMotor();
    // }
    motor.setControl(m_request.withPosition(Units.degreesToRotations(position)));
  }

  @Override
  public void resetPosition(double position) {
    motor.setPosition(Units.degreesToRotations(position));
  }

  @Override
  public void updateInputs(StretcherIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                motor.getMotorVoltage(), motor.getSupplyCurrent(), motor.getVelocity())
            .isOK();

    inputs.motorVoltageVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.stretcherRotationDegrees =
        Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    inputs.stretcherPositionRadians = inputs.stretcherRotationDegrees * Math.PI / 180.0;
  }
}
